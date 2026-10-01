#include <unity.h>
#include <initializer_list>
#include "modules/drive_supervisor.h"
#include "modules/realcar_calibration.h"

static DriveSupervisorParams params() {
    return {500.0f, 0.5f, 8000.0f, 0.92f, 0.1266f,
            500.0f,
            8000.0f, 200.0f, 150.0f};
}

static DriveSupervisorState supervisor_state{};

static DriveSupervisorOutput compute(
    const DriveSupervisorInput &in, const DriveSupervisorParams &p) {
    return drive_supervisor_compute(in, p, supervisor_state);
}

static DriveSupervisorOutput compute(const DriveSupervisorInput &in) {
    const DriveSupervisorParams p = params();
    return compute(in, p);
}

static DriveSupervisorInput nominal() {
    DriveSupervisorInput in{};
    in.requested_left_a = 100.0f;
    in.requested_right_a = 100.0f;
    in.controller_feedback_fresh = true;
    in.bus_voltage_left_v = 57.0f;
    in.bus_voltage_right_v = 57.0f;
    in.motor_rpm_left = 1000;
    in.motor_rpm_right = 1000;
    in.controller_temp_left_c = 40.0f;
    in.controller_temp_right_c = 40.0f;
    in.motor_temp_left_c = 50.0f;
    in.motor_temp_right_c = 50.0f;
    in.pack_data_valid = true;
    in.pack_current_a = 10.0f;
    return in;
}

void test_stale_feedback_blocks_all_current(void) {
    DriveSupervisorInput in = nominal();
    in.controller_feedback_fresh = false;
    auto out = compute(in);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);
    TEST_ASSERT_TRUE(out.controller_blocked);
}

void test_fault_blocks_all_current(void) {
    DriveSupervisorInput in = nominal();
    in.controller_fault = true;
    auto out = compute(in);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.right_a);
}

void test_power_limit_scales_both_motors(void) {
    DriveSupervisorInput in = nominal();
    in.requested_left_a = 500.0f;
    in.requested_right_a = 500.0f;
    in.phase_current_left_a = 500.0f;
    in.phase_current_right_a = 500.0f;
    in.motor_rpm_left = 2500;
    in.motor_rpm_right = 2500;
    auto out = compute(in);
    TEST_ASSERT_TRUE(out.power_limited);
    TEST_ASSERT_TRUE(out.left_a < 300.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, out.left_a, out.right_a);
}

void test_power_limit_uses_actual_phase_current_not_500_a_target(void) {
    DriveSupervisorInput in = nominal();
    in.propulsion_requested = true;
    in.control_dt_s = 0.01f;
    in.requested_left_a = 500.0f;
    in.requested_right_a = 500.0f;
    in.phase_current_left_a = 20.0f;
    in.phase_current_right_a = 20.0f;
    in.motor_rpm_left = 2500;
    in.motor_rpm_right = 2500;

    auto out = compute(in);
    TEST_ASSERT_FALSE(out.power_limited);
    TEST_ASSERT_TRUE(out.estimated_input_power_w < 8000.0f);
    TEST_ASSERT_TRUE(out.predicted_command_power_w < 8000.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, out.left_a);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, out.right_a);
}

void test_slew_limited_command_prediction_caps_before_feedback_arrives(void) {
    DriveSupervisorInput in = nominal();
    in.propulsion_requested = true;
    in.control_dt_s = 0.01f;
    in.requested_left_a = 500.0f;
    in.requested_right_a = 500.0f;
    in.phase_current_left_a = 0.0f;
    in.phase_current_right_a = 0.0f;
    in.bus_current_left_a = 0.0f;
    in.bus_current_right_a = 0.0f;
    in.motor_rpm_left = 2500;
    in.motor_rpm_right = 2500;

    DriveSupervisorOutput out{};
    for (int tick = 0; tick < 50; ++tick) out = compute(in);

    TEST_ASSERT_TRUE(out.power_limited);
    TEST_ASSERT_TRUE(out.predicted_command_power_w > 8000.0f);
    TEST_ASSERT_TRUE(out.left_a > 100.0f);
    TEST_ASSERT_TRUE(out.left_a < 112.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, out.left_a, out.right_a);
}

void test_zero_power_limit_disables_power_limiting(void) {
    DriveSupervisorInput in = nominal();
    in.requested_left_a = 300.0f;
    in.requested_right_a = 300.0f;
    in.motor_rpm_left = 2500;
    in.motor_rpm_right = 2500;
    DriveSupervisorParams p = params();
    p.power_soft_limit_w = 0.0f;
    p.drive_current_rise_time_s = 0.0f;
    auto out = compute(in, p);
    TEST_ASSERT_FALSE(out.power_limited);
    TEST_ASSERT_EQUAL_FLOAT(300.0f, out.left_a);
    TEST_ASSERT_EQUAL_FLOAT(300.0f, out.right_a);
}

void test_paddock_fixed_current_ceiling(void) {
    auto in = nominal();
    in.paddock_active = true;
    in.requested_left_a = in.requested_right_a = 600.0f;
    in.motor_rpm_left = in.motor_rpm_right = 0;
    auto p = params();
    p.paddock_current_max_per_motor_a = 100.0f;
    for (float speed : {0.0f, 1.0f, 20.0f}) {
        in.paddock_speed_mps = speed;
        auto out = compute(in, p);
        TEST_ASSERT_EQUAL_FLOAT(100.0f, out.left_a);
        TEST_ASSERT_EQUAL_FLOAT(100.0f, out.right_a);
        TEST_ASSERT_TRUE(out.paddock_current_limited);
    }
}

void test_paddock_propulsion_rises_to_500_a_in_half_second(void) {
    DriveSupervisorInput in = nominal();
    in.paddock_active = true;
    in.propulsion_requested = true;
    in.control_dt_s = 0.01f;
    in.requested_left_a = 500.0f;
    in.requested_right_a = 500.0f;
    in.motor_rpm_left = 0;
    in.motor_rpm_right = 0;

    auto out = compute(in);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, out.left_a);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, out.right_a);
    TEST_ASSERT_TRUE(out.drive_slew_limited);

    for (int tick = 1; tick < 25; ++tick) out = compute(in);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 250.0f, out.left_a);

    for (int tick = 25; tick < 50; ++tick) out = compute(in);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 500.0f, out.left_a);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 500.0f, out.right_a);
    TEST_ASSERT_FALSE(out.drive_slew_limited);
}

void test_normal_propulsion_rises_to_500_a_in_half_second(void) {
    DriveSupervisorInput in = nominal();
    in.paddock_active = false;
    in.propulsion_requested = true;
    in.control_dt_s = 0.01f;
    in.requested_left_a = 500.0f;
    in.requested_right_a = 500.0f;
    in.motor_rpm_left = 0;
    in.motor_rpm_right = 0;
    DriveSupervisorParams p = params();
    p.power_soft_limit_w = 0.0f;

    auto out = compute(in, p);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, out.left_a);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, out.right_a);
    TEST_ASSERT_TRUE(out.drive_slew_limited);

    for (int tick = 1; tick < 25; ++tick) out = compute(in, p);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 250.0f, out.left_a);

    for (int tick = 25; tick < 50; ++tick) out = compute(in, p);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 500.0f, out.left_a);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 500.0f, out.right_a);
    TEST_ASSERT_FALSE(out.drive_slew_limited);

    in.requested_left_a = 0.0f;
    in.requested_right_a = 0.0f;
    in.propulsion_requested = false;
    out = compute(in, p);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.right_a);
}

void test_paddock_release_and_fault_reductions_are_immediate(void) {
    DriveSupervisorInput in = nominal();
    in.paddock_active = true;
    in.propulsion_requested = true;
    in.control_dt_s = 0.01f;
    in.requested_left_a = 300.0f;
    in.requested_right_a = 300.0f;
    in.motor_rpm_left = 0;
    in.motor_rpm_right = 0;
    for (int tick = 0; tick < 50; ++tick) compute(in);

    in.requested_left_a = 0.0f;
    in.requested_right_a = 0.0f;
    in.propulsion_requested = false;
    auto out = compute(in);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.right_a);

    in.requested_left_a = -500.0f;
    in.requested_right_a = -500.0f;
    in.propulsion_requested = true;
    out = compute(in);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -10.0f, out.left_a);

    in.controller_fault = true;
    out = compute(in);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);
    in.controller_fault = false;
    out = compute(in);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -10.0f, out.left_a);
}

void test_paddock_ignores_missing_temperature_and_pack(void) {
    DriveSupervisorInput in = nominal();
    in.paddock_active = true;
    in.motor_temp_left_c = -40.0f;
    auto out = compute(in);
    TEST_ASSERT_FALSE(out.paddock_sensor_blocked);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, out.left_a);

    in = nominal();
    in.paddock_active = true;
    in.pack_data_valid = false;
    out = compute(in);
    TEST_ASSERT_FALSE(out.paddock_sensor_blocked);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, out.right_a);
}

void test_paddock_pack_current_and_power_guards_scale_drive(void) {
    DriveSupervisorInput in = nominal();
    in.paddock_active = true;
    in.pack_current_a = 300.0f;
    auto out = compute(in);
    TEST_ASSERT_TRUE(out.paddock_current_limited);
    TEST_ASSERT_TRUE(out.left_a < 100.0f);

    in = nominal();
    in.paddock_active = true;
    in.bus_voltage_left_v = 57.0f;
    in.bus_voltage_right_v = 57.0f;
    in.bus_current_left_a = 80.0f;
    in.bus_current_right_a = 80.0f;
    out = compute(in);
    TEST_ASSERT_TRUE(out.power_limited);
    TEST_ASSERT_TRUE(out.left_a < 100.0f);
}

void test_temperature_does_not_limit_drive_or_regen(void) {
    for (bool paddock : {false, true}) {
        for (float demand : {100.0f, -20.0f}) {
            for (float temperature : {-40.0f, 80.0f, 125.0f, 200.0f}) {
                DriveSupervisorInput in = nominal();
                in.paddock_active = paddock;
                in.requested_left_a = in.requested_right_a = demand;
                in.controller_temp_left_c = in.controller_temp_right_c = temperature;
                in.motor_temp_left_c = in.motor_temp_right_c = temperature;
                auto out = compute(in);
                TEST_ASSERT_FALSE(out.thermal_limited);
                TEST_ASSERT_EQUAL_FLOAT(demand, out.left_a);
                TEST_ASSERT_EQUAL_FLOAT(demand, out.right_a);
            }
        }
    }
}

void test_normal_drive_does_not_require_pack(void) {
    auto in = nominal();
    in.pack_data_valid = false;
    auto out = compute(in);
    TEST_ASSERT_FALSE(out.paddock_sensor_blocked);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, out.left_a);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, out.right_a);
}

static DriveSupervisorParams rpm_cap_params() {
    auto p = params();
    p.drive_current_rise_time_s = 0.0f;
    p.power_soft_limit_w = 0.0f;
    p.rpm_cap_power_per_motor_w = 4600.0f;
    p.rpm_cap_w_per_a_offset = 4.676f;
    p.rpm_cap_w_per_a_per_rpm = 7.68e-3f;
    return p;
}

void test_rpm_cap_limits_phase_current_at_speed(void) {
    auto in = nominal();
    in.propulsion_requested = true;
    in.requested_left_a = in.requested_right_a = 480.0f;
    in.motor_rpm_left = 1800;
    in.motor_rpm_right = -1800;  // right motor spins negative
    auto out = compute(in, rpm_cap_params());
    const float cap = 4600.0f / (4.676f + 7.68e-3f * 1800.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, cap, out.left_a);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, cap, out.right_a);
    TEST_ASSERT_TRUE(out.rpm_cap_limited);
}

void test_rpm_cap_leaves_launch_current_alone(void) {
    auto in = nominal();
    in.propulsion_requested = true;
    in.requested_left_a = in.requested_right_a = 480.0f;
    in.motor_rpm_left = in.motor_rpm_right = 300;
    auto out = compute(in, rpm_cap_params());
    TEST_ASSERT_EQUAL_FLOAT(480.0f, out.left_a);
    TEST_ASSERT_EQUAL_FLOAT(480.0f, out.right_a);
    TEST_ASSERT_FALSE(out.rpm_cap_limited);
}

void test_rpm_cap_keeps_torque_vectoring_split(void) {
    auto in = nominal();
    in.propulsion_requested = true;
    in.requested_left_a = 400.0f;
    in.requested_right_a = 200.0f;
    in.motor_rpm_left = in.motor_rpm_right = 1800;
    auto out = compute(in, rpm_cap_params());
    TEST_ASSERT_TRUE(out.rpm_cap_limited);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, out.left_a / out.right_a);
}

void test_rpm_cap_applies_in_reverse(void) {
    auto in = nominal();
    in.propulsion_requested = true;
    in.requested_left_a = in.requested_right_a = -480.0f;
    in.motor_rpm_left = in.motor_rpm_right = -1800;
    auto out = compute(in, rpm_cap_params());
    TEST_ASSERT_TRUE(out.rpm_cap_limited);
    TEST_ASSERT_TRUE(out.left_a < 0.0f && out.left_a > -260.0f);
}

void test_rpm_cap_does_not_touch_regen(void) {
    auto in = nominal();
    in.propulsion_requested = false;
    in.requested_left_a = in.requested_right_a = -480.0f;
    in.motor_rpm_left = in.motor_rpm_right = 1800;
    auto out = compute(in, rpm_cap_params());
    TEST_ASSERT_FALSE(out.rpm_cap_limited);
    TEST_ASSERT_EQUAL_FLOAT(-480.0f, out.left_a);
}

void test_rpm_cap_zero_target_disables(void) {
    auto p = rpm_cap_params();
    p.rpm_cap_power_per_motor_w = 0.0f;
    auto in = nominal();
    in.propulsion_requested = true;
    in.requested_left_a = in.requested_right_a = 480.0f;
    in.motor_rpm_left = in.motor_rpm_right = 1800;
    auto out = compute(in, p);
    TEST_ASSERT_FALSE(out.rpm_cap_limited);
    TEST_ASSERT_EQUAL_FLOAT(480.0f, out.left_a);
}

void test_rpm_cap_ramp_aims_at_cap(void) {
    auto p = rpm_cap_params();
    p.drive_current_rise_time_s = 0.5f;
    auto in = nominal();
    in.propulsion_requested = true;
    in.control_dt_s = 0.01f;
    in.requested_left_a = in.requested_right_a = 480.0f;
    in.motor_rpm_left = in.motor_rpm_right = 1800;
    const float cap = 4600.0f / (4.676f + 7.68e-3f * 1800.0f);
    DriveSupervisorOutput out{};
    for (int i = 0; i < 100; ++i) {
        out = compute(in, p);
        TEST_ASSERT_TRUE(out.left_a <= cap + 0.01f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, cap, out.left_a);
}

void setUp(void) { supervisor_state = DriveSupervisorState{}; }
void test_current_profile_electrical_feedback_does_not_limit_drive(void) {
    auto p = params();
    using namespace realcar_cal::bringup;
    // This test isolates electrical-feedback limits, not the launch ramp.
    p.drive_current_rise_time_s = 0.0f;
    p.power_soft_limit_w = ENABLE_DRIVE_POWER_LIMIT ? DRIVE_POWER_SOFT_LIMIT_W : 0.0f;
    p.paddock_power_soft_limit_w = PADDOCK_POWER_SOFT_LIMIT_W;
    p.paddock_controller_bus_current_limit_a = PADDOCK_CONTROLLER_BUS_CURRENT_LIMIT_A;
    p.paddock_pack_current_limit_a = PADDOCK_PACK_CURRENT_LIMIT_A;
    p.paddock_current_max_per_motor_a = PADDOCK_CURRENT_MAX_PER_MOTOR_A;
    const bool modes[] = {false, true};
    for (bool paddock : modes) {
        auto in = nominal();
        in.paddock_active = paddock;
        in.propulsion_requested = true;
        in.bus_voltage_left_v = 20.0f;
        in.bus_voltage_right_v = 80.0f;
        in.bus_current_left_a = 2000.0f;
        in.bus_current_right_a = 2000.0f;
        in.phase_current_left_a = 1500.0f;
        in.phase_current_right_a = 1500.0f;
        in.pack_current_a = 1000.0f;
        auto out = compute(in, p);
        TEST_ASSERT_EQUAL_FLOAT(100.0f, out.left_a);
        TEST_ASSERT_EQUAL_FLOAT(100.0f, out.right_a);
        TEST_ASSERT_FALSE(out.controller_blocked);
        TEST_ASSERT_FALSE(out.power_limited);
        TEST_ASSERT_FALSE(out.paddock_current_limited);
        TEST_ASSERT_TRUE(out.measured_bus_power_w > 10000.0f);
    }
}
void test_regen_rpm_power_cap(void) {
    auto p = params();
    p.power_soft_limit_w = 0.0f;
    p.regen_mechanical_power_per_motor_w = 4000.0f;
    auto in = nominal();
    in.requested_left_a = -500.0f;
    in.requested_right_a = -500.0f;
    in.motor_rpm_left = 2000;
    in.motor_rpm_right = -2000;
    auto out = compute(in, p);
    const float cap = 4000.0f / (0.1266f * 2000.0f * 0.104719755f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -cap, out.left_a);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -cap, out.right_a);
    TEST_ASSERT_TRUE(out.power_limited);
    in.motor_rpm_left = 100;
    in.motor_rpm_right = -100;
    out = compute(in, p);
    TEST_ASSERT_EQUAL_FLOAT(-500.0f, out.left_a);
    in.motor_rpm_right = 0;
    out = compute(in, p);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.right_a);
    in.propulsion_requested = true;
    p.drive_current_rise_time_s = 0.0f;
    out = compute(in, p);
    TEST_ASSERT_EQUAL_FLOAT(-500.0f, out.left_a);
    in.propulsion_requested = false;
    in.controller_fault = true;
    out = compute(in, p);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);
}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_regen_rpm_power_cap);
    RUN_TEST(test_current_profile_electrical_feedback_does_not_limit_drive);
    RUN_TEST(test_stale_feedback_blocks_all_current);
    RUN_TEST(test_fault_blocks_all_current);
    RUN_TEST(test_power_limit_scales_both_motors);
    RUN_TEST(test_power_limit_uses_actual_phase_current_not_500_a_target);
    RUN_TEST(test_slew_limited_command_prediction_caps_before_feedback_arrives);
    RUN_TEST(test_zero_power_limit_disables_power_limiting);
    RUN_TEST(test_paddock_fixed_current_ceiling);
    RUN_TEST(test_paddock_propulsion_rises_to_500_a_in_half_second);
    RUN_TEST(test_normal_propulsion_rises_to_500_a_in_half_second);
    RUN_TEST(test_paddock_release_and_fault_reductions_are_immediate);
    RUN_TEST(test_paddock_ignores_missing_temperature_and_pack);
    RUN_TEST(test_normal_drive_does_not_require_pack);
    RUN_TEST(test_paddock_pack_current_and_power_guards_scale_drive);
    RUN_TEST(test_temperature_does_not_limit_drive_or_regen);
    RUN_TEST(test_rpm_cap_limits_phase_current_at_speed);
    RUN_TEST(test_rpm_cap_leaves_launch_current_alone);
    RUN_TEST(test_rpm_cap_keeps_torque_vectoring_split);
    RUN_TEST(test_rpm_cap_applies_in_reverse);
    RUN_TEST(test_rpm_cap_does_not_touch_regen);
    RUN_TEST(test_rpm_cap_zero_target_disables);
    RUN_TEST(test_rpm_cap_ramp_aims_at_cap);
    return UNITY_END();
}
