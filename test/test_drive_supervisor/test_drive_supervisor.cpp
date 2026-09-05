#include <unity.h>
#include "modules/drive_supervisor.h"

static DriveSupervisorParams params() {
    return {9000.0f, 0.92f, 0.1266f, 30.0f, 2.2222f, 2.7778f,
            3500.0f, 90.0f, 70.0f, -30.0f, true,
            75.0f, 85.0f, 100.0f, 120.0f};
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
    auto out = drive_supervisor_compute(in, params());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);
    TEST_ASSERT_TRUE(out.controller_blocked);
}

void test_fault_blocks_all_current(void) {
    DriveSupervisorInput in = nominal();
    in.controller_fault = true;
    auto out = drive_supervisor_compute(in, params());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.right_a);
}

void test_power_limit_scales_both_motors(void) {
    DriveSupervisorInput in = nominal();
    in.requested_left_a = 300.0f;
    in.requested_right_a = 300.0f;
    in.motor_rpm_left = 2500;
    in.motor_rpm_right = 2500;
    auto out = drive_supervisor_compute(in, params());
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
    auto out = drive_supervisor_compute(in, p);
    TEST_ASSERT_FALSE(out.power_limited);
    TEST_ASSERT_EQUAL_FLOAT(300.0f, out.left_a);
    TEST_ASSERT_EQUAL_FLOAT(300.0f, out.right_a);
}

void test_paddock_clamps_current_and_speed(void) {
    DriveSupervisorInput in = nominal();
    in.paddock_active = true;
    auto out = drive_supervisor_compute(in, params());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, out.left_a);
    in.paddock_speed_mps = 3.0f;
    out = drive_supervisor_compute(in, params());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);
}

void test_paddock_tapers_before_speed_limit(void) {
    DriveSupervisorInput in = nominal();
    in.paddock_active = true;
    in.paddock_speed_mps = 2.5f;
    auto out = drive_supervisor_compute(in, params());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 15.0f, out.left_a);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.15f, out.applied_scale);
}

void test_paddock_blocks_missing_temperature_or_pack_data(void) {
    DriveSupervisorInput in = nominal();
    in.paddock_active = true;
    in.motor_temp_left_c = -40.0f;
    auto out = drive_supervisor_compute(in, params());
    TEST_ASSERT_TRUE(out.paddock_sensor_blocked);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);

    in = nominal();
    in.paddock_active = true;
    in.pack_data_valid = false;
    out = drive_supervisor_compute(in, params());
    TEST_ASSERT_TRUE(out.paddock_sensor_blocked);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.right_a);
}

void test_paddock_pack_current_and_power_guards_scale_drive(void) {
    DriveSupervisorInput in = nominal();
    in.paddock_active = true;
    in.pack_current_a = 140.0f;
    auto out = drive_supervisor_compute(in, params());
    TEST_ASSERT_TRUE(out.paddock_current_limited);
    TEST_ASSERT_TRUE(out.left_a < 30.0f);

    in = nominal();
    in.paddock_active = true;
    in.bus_voltage_left_v = 57.0f;
    in.bus_voltage_right_v = 57.0f;
    in.bus_current_left_a = 40.0f;
    in.bus_current_right_a = 40.0f;
    out = drive_supervisor_compute(in, params());
    TEST_ASSERT_TRUE(out.power_limited);
    TEST_ASSERT_TRUE(out.left_a < 30.0f);
}

void test_paddock_timeout_blocks_drive(void) {
    DriveSupervisorInput in = nominal();
    in.paddock_active = true;
    in.paddock_timed_out = true;
    auto out = drive_supervisor_compute(in, params());
    TEST_ASSERT_TRUE(out.paddock_timed_out);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);
}

void test_thermal_cutoff_also_blocks_regen(void) {
    DriveSupervisorInput in = nominal();
    in.requested_left_a = -20.0f;
    in.requested_right_a = -20.0f;
    in.controller_temp_left_c = 85.0f;
    auto out = drive_supervisor_compute(in, params());
    TEST_ASSERT_TRUE(out.thermal_limited);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.right_a);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_stale_feedback_blocks_all_current);
    RUN_TEST(test_fault_blocks_all_current);
    RUN_TEST(test_power_limit_scales_both_motors);
    RUN_TEST(test_zero_power_limit_disables_power_limiting);
    RUN_TEST(test_paddock_clamps_current_and_speed);
    RUN_TEST(test_paddock_tapers_before_speed_limit);
    RUN_TEST(test_paddock_blocks_missing_temperature_or_pack_data);
    RUN_TEST(test_paddock_pack_current_and_power_guards_scale_drive);
    RUN_TEST(test_paddock_timeout_blocks_drive);
    RUN_TEST(test_thermal_cutoff_also_blocks_regen);
    return UNITY_END();
}
