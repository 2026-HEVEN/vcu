#include <unity.h>
#include "modules/drive_supervisor.h"

static DriveSupervisorParams params() {
    return {9000.0f, 0.92f, 0.1266f, 30.0f, 2.7778f,
            2.0f, 0.12f, 0.25f, 75.0f, 85.0f, 100.0f, 120.0f};
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

void test_paddock_clamps_current_and_speed(void) {
    DriveSupervisorInput in = nominal();
    in.paddock_active = true;
    auto out = drive_supervisor_compute(in, params());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, out.left_a);
    in.vehicle_speed_mps = 3.0f;
    out = drive_supervisor_compute(in, params());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);
}

void test_tc_reduces_only_slipping_side(void) {
    DriveSupervisorInput in = nominal();
    in.tc_enabled = true;
    in.vehicle_speed_valid = true;
    in.vehicle_speed_mps = 5.0f;
    in.wheel_rpm_fl = 100.0f;
    in.wheel_rpm_fr = 100.0f;
    in.wheel_rpm_rl = 130.0f;
    in.wheel_rpm_rr = 100.0f;
    auto out = drive_supervisor_compute(in, params());
    TEST_ASSERT_TRUE(out.traction_limited);
    TEST_ASSERT_TRUE(out.left_a < out.right_a);
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
    RUN_TEST(test_paddock_clamps_current_and_speed);
    RUN_TEST(test_tc_reduces_only_slipping_side);
    RUN_TEST(test_thermal_cutoff_also_blocks_regen);
    return UNITY_END();
}
