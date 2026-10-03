#include <unity.h>
#include <cmath>
#include <initializer_list>
#include "modules/motor_direction.h"
#include "modules/realcar_calibration.h"
#include "modules/fixed_config.h"
#include "safety_logic.h"
#include "can_protocol.h"

static MotorDirectionCommand request(float pedal, Gear gear) {
    return paddock_motor_request(pedal, gear,
        realcar_cal::bringup::PADDOCK_MAX_SPEED_KPH,
        realcar_cal::bringup::PADDOCK_CURRENT_MAX_PER_MOTOR_A,
        realcar_cal::provisional::WHEEL_SPEED_ROLLING_RADIUS_M,
        fixed_config::vehicle::GEAR_RATIO);
}
static MotorCommandSnapshot snapshot(float pedal, Gear gear) {
    const auto r = request(pedal, gear);
    MotorCommandSnapshot s;
    s.left_a = s.right_a = r.current_a;
    s.propulsion_rpm_limit = std::abs(r.target_rpm);
    s.gear = gear;
    s.safety_allow = s.throttle_signal_valid = s.propulsion_direction_armed = true;
    return s;
}
static MotorCommandGates gates() {
    MotorCommandGates g;
    g.scheduler_alive = g.snapshot_fresh = true;
    g.reconnect_inhibit = g.component_test_inhibit = false;
    return g;
}
static float rpm_to_kph(int rpm) {
    return std::abs(rpm) * 6.283185307f *
        realcar_cal::provisional::WHEEL_SPEED_ROLLING_RADIUS_M * 3.6f /
        (60.0f * fixed_config::vehicle::GEAR_RATIO);
}
void test_pedal_scales_current_and_target_is_fixed_cap() {
    const float max_a = realcar_cal::bringup::PADDOCK_CURRENT_MAX_PER_MOTOR_A;
    for (Gear gear : {Gear::Drive, Gear::Reverse}) {
        const int sign = gear == Gear::Drive ? 1 : -1;
        const float cap = gear == Gear::Drive
            ? realcar_cal::bringup::PADDOCK_MAX_SPEED_KPH
            : realcar_cal::bringup::PADDOCK_REVERSE_MAX_SPEED_KPH;
        const int cap_rpm = request(100, gear).target_rpm;
        for (float pedal : {1.0f, 25.0f, 50.0f, 100.0f, 150.0f}) {
            const auto r = request(pedal, gear);
            TEST_ASSERT_FLOAT_WITHIN(1e-3f,
                sign * max_a * std::fmin(pedal, 100.0f) / 100.0f, r.current_a);
            // A partial pedal never lowers the target below the cap.
            TEST_ASSERT_EQUAL_INT(cap_rpm, r.target_rpm);
            TEST_ASSERT_TRUE(r.target_rpm * sign > 0);
            TEST_ASSERT_FLOAT_WITHIN(0.03f, cap, rpm_to_kph(r.target_rpm));
            TEST_ASSERT_TRUE(rpm_to_kph(r.target_rpm) <= cap);
        }
    }
}
void test_drive_cap_is_30_kph_and_reverse_is_5_kph() {
    TEST_ASSERT_EQUAL_INT(1240, request(100, Gear::Drive).target_rpm);
    TEST_ASSERT_EQUAL_INT(-206, request(100, Gear::Reverse).target_rpm);
}
void test_release_invalid_and_neutral_have_no_hold_current() {
    for (float pedal : {0.0f, -1.0f, 0.01f, NAN}) {
        const auto r = request(pedal, Gear::Drive);
        TEST_ASSERT_EQUAL_FLOAT(0, r.current_a);
        TEST_ASSERT_EQUAL_INT(0, r.target_rpm);
    }
    TEST_ASSERT_EQUAL_FLOAT(0, request(100, Gear::Neutral).current_a);
}
void test_snapshot_can_bytes_and_recovery_ramp() {
    for (Gear gear : {Gear::Drive, Gear::Reverse}) {
        auto g = gates();
        g.reconnect_ramp_scale = 0.5f;
        const auto c = motor_command_resolve(snapshot(100, gear), g);
        const int sign = gear == Gear::Drive ? 1 : -1;
        TEST_ASSERT_EQUAL_FLOAT(sign * 0.5f *
            realcar_cal::bringup::PADDOCK_CURRENT_MAX_PER_MOTOR_A, c.left_a);
        TEST_ASSERT_EQUAL_FLOAT(c.left_a, c.right_a);
        TEST_ASSERT_EQUAL_INT(request(100, gear).target_rpm, c.target_rpm_L);
        TEST_ASSERT_EQUAL_INT(c.target_rpm_L, c.target_rpm_R);
        uint8_t data[8];
        encode_motor_control(c.left_a, c.target_rpm_L, c.run_L, 9, data);
        TEST_ASSERT_EQUAL_HEX8(0x01, data[4]); // no mode-bit/app change
        TEST_ASSERT_EQUAL_INT(c.target_rpm_L, raw_to_speed(data[2] | (data[3] << 8)));
        TEST_ASSERT_EQUAL_FLOAT(c.left_a, raw_to_current(data[0] | (data[1] << 8)));
    }
}
void test_fault_and_stale_still_block_both() {
    auto g = gates();
    g.controller_fault_active = true;
    auto c = motor_command_resolve(snapshot(100, Gear::Drive), g);
    TEST_ASSERT_FALSE(c.normal_allow);
    TEST_ASSERT_EQUAL_FLOAT(0, c.left_a);
    g = gates(); g.snapshot_fresh = false;
    c = motor_command_resolve(snapshot(100, Gear::Drive), g);
    TEST_ASSERT_FALSE(c.run_L);
    TEST_ASSERT_FALSE(c.run_R);
}
void test_regen_permission_is_not_bypassed_by_rpm_limit() {
    auto s = snapshot(0, Gear::Drive);
    s.left_a = s.right_a = -20;
    s.forward_rotation = true;
    auto c = motor_command_resolve(s, gates());
    TEST_ASSERT_EQUAL_FLOAT(0, c.left_a);
    s.regen_allowed = true;
    c = motor_command_resolve(s, gates());
    TEST_ASSERT_EQUAL_FLOAT(-20, c.left_a);
    TEST_ASSERT_EQUAL_INT(0, c.target_rpm_L);
}
void test_paddock_exit_restores_normal_rpm() {
    auto s = snapshot(100, Gear::Drive);
    s.propulsion_rpm_limit = DRIVE_TARGET_SPEED_RPM;
    TEST_ASSERT_EQUAL_INT(4000, motor_command_resolve(s, gates()).target_rpm_L);
    s.propulsion_rpm_limit = -1;
    TEST_ASSERT_FALSE(motor_command_resolve(s, gates()).normal_allow);
}
void setUp() {}
void tearDown() {}
int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_pedal_scales_current_and_target_is_fixed_cap);
    RUN_TEST(test_drive_cap_is_30_kph_and_reverse_is_5_kph);
    RUN_TEST(test_release_invalid_and_neutral_have_no_hold_current);
    RUN_TEST(test_snapshot_can_bytes_and_recovery_ramp);
    RUN_TEST(test_fault_and_stale_still_block_both);
    RUN_TEST(test_regen_permission_is_not_bypassed_by_rpm_limit);
    RUN_TEST(test_paddock_exit_restores_normal_rpm);
    return UNITY_END();
}
