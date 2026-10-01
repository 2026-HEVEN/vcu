#include <unity.h>
#include <cmath>
#include "modules/longitudinal.h"
#include "modules/realcar_calibration.h"

void setUp(void) {}
void tearDown(void) {}

void test_full_throttle_requests_configured_current_for_both_motors(void) {
    const float total = longitudinal_compute(
        {100.0f, 0.5f, DriveMode::Normal, true});
    TEST_ASSERT_FLOAT_WITHIN(
        0.01f,
        2.0f * realcar_cal::bringup::DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A,
        total);
}

void test_throttle_off_regens_without_brake_input(void) {
    TEST_ASSERT_EQUAL_FLOAT(
        -realcar_cal::bringup::REGEN_LEVEL3_TOTAL_CURRENT_A,
        longitudinal_compute({0.0f, 0.5f, DriveMode::Normal, true}));
    TEST_ASSERT_EQUAL_FLOAT(
        -30.0f, longitudinal_compute({0.0f, 0.5f, DriveMode::Efficiency, true}));
}

void test_regen_taper_midpoint(void) {
    const float current = longitudinal_compute(
        {0.0f, 0.925f, DriveMode::Normal, true});
    TEST_ASSERT_FLOAT_WITHIN(0.5f,
        -0.5f * realcar_cal::bringup::REGEN_LEVEL3_TOTAL_CURRENT_A, current);
}

void test_regen_cutoff_high_soc(void) {
    TEST_ASSERT_EQUAL_FLOAT(
        0.0f, longitudinal_compute({0.0f, 0.96f, DriveMode::Normal, true}));
}

void test_positive_throttle_immediately_cancels_regen(void) {
    TEST_ASSERT_TRUE(longitudinal_compute(
        {0.01f, 0.5f, DriveMode::Normal, true}) > 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(
        2.0f * realcar_cal::bringup::DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A,
        longitudinal_compute({100.0f, 0.5f, DriveMode::Normal, true}));
}

void test_release_qualification_and_immediate_cancel(void) {
    RegenReleaseState state{};
    for (int i = 0; i < 9; ++i) TEST_ASSERT_FALSE(regen_release_update(0, true, state));
    TEST_ASSERT_TRUE(regen_release_update(0, true, state));
    TEST_ASSERT_FALSE(regen_release_update(0.01f, true, state));
    TEST_ASSERT_FALSE(regen_release_update(0, true, state));
    TEST_ASSERT_FALSE(regen_release_update(0, false, state));
    TEST_ASSERT_EQUAL_UINT(0, state.zero_samples);
    TEST_ASSERT_FALSE(regen_release_update(NAN, true, state));
}

void test_invalid_inputs_do_not_request_regen(void) {
    TEST_ASSERT_EQUAL_FLOAT(0, longitudinal_compute({NAN, 0.5f, DriveMode::Normal, true}));
    TEST_ASSERT_EQUAL_FLOAT(0, longitudinal_compute({0, NAN, DriveMode::Normal, true}));
    TEST_ASSERT_EQUAL_FLOAT(0, longitudinal_compute({0, -0.1f, DriveMode::Normal, true}));
}

void test_regen_switch_off_coasts(void) {
    TEST_ASSERT_EQUAL_FLOAT(
        0.0f, longitudinal_compute({0.0f, 0.5f, DriveMode::Normal, false}));
}

void test_efficiency_limits_drive(void) {
    const float efficiency = longitudinal_compute(
        {100.0f, 0.5f, DriveMode::Efficiency, true});
    const float normal = longitudinal_compute(
        {100.0f, 0.5f, DriveMode::Normal, true});
    TEST_ASSERT_TRUE(efficiency < normal);
}

void test_stages_and_trigger_modes() {
    const float currents[] = {0, realcar_cal::bringup::REGEN_LEVEL1_TOTAL_CURRENT_A,
        realcar_cal::bringup::REGEN_LEVEL2_TOTAL_CURRENT_A,
        realcar_cal::bringup::REGEN_LEVEL3_TOTAL_CURRENT_A};
    for (unsigned level=0; level<4; ++level) {
        TEST_ASSERT_FLOAT_WITHIN(.01f, -currents[level],
            longitudinal_compute({0,.5f,DriveMode::Normal,true,level,false,true}));
        TEST_ASSERT_EQUAL_FLOAT(0,
            longitudinal_compute({0,.5f,DriveMode::Normal,true,level,false,false}));
        TEST_ASSERT_FLOAT_WITHIN(.01f, -currents[level],
            longitudinal_compute({0,.5f,DriveMode::Normal,true,level,true,false}));
        TEST_ASSERT_TRUE(longitudinal_compute({1,.5f,DriveMode::Normal,true,level,true,false})>0);
        TEST_ASSERT_EQUAL_FLOAT(0,longitudinal_compute({0,.96f,DriveMode::Normal,true,level,true,false}));
        TEST_ASSERT_EQUAL_FLOAT(0,longitudinal_compute({0,.5f,DriveMode::Normal,false,level,true,false}));
        TEST_ASSERT_FLOAT_WITHIN(.01f,level ? -currents[3] : 0,
            longitudinal_compute({0,.5f,DriveMode::Normal,true,level,false,true,false}));
    }
    TEST_ASSERT_EQUAL_FLOAT(0,longitudinal_compute({0,.5f,DriveMode::Normal,true,4,true,true}));
}

void test_regen_voltage_cutoff() {
    TEST_ASSERT_TRUE(regen_voltage_ok(57.7f,true,57.7f,true));
    TEST_ASSERT_FALSE(regen_voltage_ok(57.8f,true,55.f,true));
    TEST_ASSERT_FALSE(regen_voltage_ok(55.f,true,57.8f,true));
    TEST_ASSERT_FALSE(regen_voltage_ok(59.f,true,55.f,true));
    TEST_ASSERT_TRUE(regen_voltage_ok(59.f,false,55.f,true));
    TEST_ASSERT_TRUE(regen_voltage_ok(55.f,true,59.f,false));
    TEST_ASSERT_FALSE(regen_voltage_ok(55.f,false,55.f,false));
    TEST_ASSERT_FALSE(regen_voltage_ok(NAN,true,55.f,true));
    TEST_ASSERT_FALSE(regen_voltage_ok(55.f,true,INFINITY,true));
    TEST_ASSERT_FALSE(regen_voltage_ok(0.f,true,55.f,true));
    TEST_ASSERT_TRUE(regen_voltage_ok(57.7f,true,55.f,true));
    for (unsigned level=1; level<=3; ++level) {
        const bool allowed = regen_voltage_ok(58.f,true,55.f,true);
        TEST_ASSERT_EQUAL_FLOAT(0, longitudinal_compute({0,.5f,DriveMode::Normal,allowed,level,true,true}));
        TEST_ASSERT_TRUE(longitudinal_compute({50,.5f,DriveMode::Normal,allowed,level,true,true})>0);
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_regen_voltage_cutoff);
    RUN_TEST(test_stages_and_trigger_modes);
    RUN_TEST(test_full_throttle_requests_configured_current_for_both_motors);
    RUN_TEST(test_throttle_off_regens_without_brake_input);
    RUN_TEST(test_regen_taper_midpoint);
    RUN_TEST(test_regen_cutoff_high_soc);
    RUN_TEST(test_positive_throttle_immediately_cancels_regen);
    RUN_TEST(test_release_qualification_and_immediate_cancel);
    RUN_TEST(test_invalid_inputs_do_not_request_regen);
    RUN_TEST(test_regen_switch_off_coasts);
    RUN_TEST(test_efficiency_limits_drive);
    return UNITY_END();
}
