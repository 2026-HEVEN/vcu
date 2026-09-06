#include <unity.h>
#include "modules/drive_supervisor.h"

static DriveSupervisorParams params() {
    return {500.0f, 0.5f, 9000.0f, 0.92f, 0.1266f,
            500.0f, 50.0f, 22.2222f,
            8000.0f, 200.0f, 150.0f, -30.0f, true,
            75.0f, 85.0f, 100.0f, 120.0f};
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
    in.motor_rpm_left = 2500;
    in.motor_rpm_right = 2500;
    auto out = compute(in);
    TEST_ASSERT_TRUE(out.power_limited);
    TEST_ASSERT_TRUE(out.left_a < 300.0f);
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

void test_paddock_current_limit_decreases_linearly_with_speed(void) {
    DriveSupervisorInput in = nominal();
    in.paddock_active = true;
    in.requested_left_a = 600.0f;
    in.requested_right_a = 600.0f;
    in.motor_rpm_left = 0;
    in.motor_rpm_right = 0;
    DriveSupervisorParams p = params();
    p.drive_current_rise_time_s = 0.0f;
    auto out = compute(in, p);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 500.0f, out.left_a);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 500.0f, out.paddock_current_limit_a);

    in.paddock_speed_mps = 20.0f / 3.6f;
    out = compute(in, p);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 387.5f, out.left_a);

    in.paddock_speed_mps = 40.0f / 3.6f;
    out = compute(in, p);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 275.0f, out.left_a);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 275.0f, out.paddock_current_limit_a);

    in.paddock_speed_mps = 60.0f / 3.6f;
    out = compute(in, p);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 162.5f, out.left_a);

    in.paddock_speed_mps = 80.0f / 3.6f;
    out = compute(in, p);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 50.0f, out.left_a);

    in.paddock_speed_mps = 120.0f / 3.6f;
    out = compute(in, p);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 50.0f, out.left_a);
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

void test_paddock_blocks_missing_temperature_or_pack_data(void) {
    DriveSupervisorInput in = nominal();
    in.paddock_active = true;
    in.motor_temp_left_c = -40.0f;
    auto out = compute(in);
    TEST_ASSERT_TRUE(out.paddock_sensor_blocked);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);

    in = nominal();
    in.paddock_active = true;
    in.pack_data_valid = false;
    out = compute(in);
    TEST_ASSERT_TRUE(out.paddock_sensor_blocked);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.right_a);
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

void test_thermal_cutoff_also_blocks_regen(void) {
    DriveSupervisorInput in = nominal();
    in.requested_left_a = -20.0f;
    in.requested_right_a = -20.0f;
    in.controller_temp_left_c = 85.0f;
    auto out = compute(in);
    TEST_ASSERT_TRUE(out.thermal_limited);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.right_a);
}

void setUp(void) { supervisor_state = DriveSupervisorState{}; }
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_stale_feedback_blocks_all_current);
    RUN_TEST(test_fault_blocks_all_current);
    RUN_TEST(test_power_limit_scales_both_motors);
    RUN_TEST(test_zero_power_limit_disables_power_limiting);
    RUN_TEST(test_paddock_current_limit_decreases_linearly_with_speed);
    RUN_TEST(test_paddock_propulsion_rises_to_500_a_in_half_second);
    RUN_TEST(test_normal_propulsion_rises_to_500_a_in_half_second);
    RUN_TEST(test_paddock_release_and_fault_reductions_are_immediate);
    RUN_TEST(test_paddock_blocks_missing_temperature_or_pack_data);
    RUN_TEST(test_paddock_pack_current_and_power_guards_scale_drive);
    RUN_TEST(test_thermal_cutoff_also_blocks_regen);
    return UNITY_END();
}
