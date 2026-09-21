#include <unity.h>
#include <cmath>
#include "modules/longitudinal.h"
#include "modules/realcar_calibration.h"

void test_full_throttle_requests_configured_current_for_both_motors(void) {
    const float total = longitudinal_compute(
        {100.0f, 0.0f, 0.5f, DriveMode::Normal, true});
    TEST_ASSERT_FLOAT_WITHIN(
        0.01f,
        2.0f * realcar_cal::bringup::DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A,
        total);
}

void test_regen_taper_midpoint(void) {
    float t = longitudinal_compute({0.0f, 100.0f, 0.925f, DriveMode::Normal, true});
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -10.0f, t);
}

void test_regen_cutoff_high_soc(void) {
    TEST_ASSERT_EQUAL_FLOAT(
        0.0f,
        longitudinal_compute({0.0f, 100.0f, 0.96f, DriveMode::Normal, true})
    );
}

void test_throttle_wins_over_brake(void) {
    float t = longitudinal_compute(
        {100.0f, 10.0f, 0.5f, DriveMode::Normal, true});

    TEST_ASSERT_EQUAL_FLOAT(1000.0f, t);
    TEST_ASSERT_EQUAL_FLOAT(t, longitudinal_compute({100, 100, 0.5f, DriveMode::Normal, false}));
    TEST_ASSERT_TRUE(longitudinal_compute({0.01f, 100, 0.5f, DriveMode::Normal, true}) > 0);
}

void test_release_qualification_and_immediate_cancel() {
    RegenReleaseState state{};
    for (int i = 0; i < 9; ++i) TEST_ASSERT_FALSE(regen_release_update(0, true, state));
    TEST_ASSERT_TRUE(regen_release_update(0, true, state));
    TEST_ASSERT_FALSE(regen_release_update(0.01f, true, state));
    TEST_ASSERT_FALSE(regen_release_update(0, true, state));
    TEST_ASSERT_FALSE(regen_release_update(0, false, state));
    TEST_ASSERT_EQUAL_UINT(0, state.zero_samples);
    TEST_ASSERT_FALSE(regen_release_update(NAN, true, state));
}

void test_invalid_inputs_do_not_request_regen() {
    TEST_ASSERT_EQUAL_FLOAT(0, longitudinal_compute({NAN, 100, 0.5f, DriveMode::Normal, true}));
    TEST_ASSERT_EQUAL_FLOAT(0, longitudinal_compute({0, 100, NAN, DriveMode::Normal, true}));
    TEST_ASSERT_EQUAL_FLOAT(0, longitudinal_compute({0, NAN, 0.5f, DriveMode::Normal, true}));
}

void test_efficiency_limits_drive(void) {
    float e =
        longitudinal_compute(
            {100.0f,0.0f,0.5f,DriveMode::Efficiency, true});

    float n =
        longitudinal_compute(
            {100.0f,0.0f,0.5f,DriveMode::Normal, true});

    TEST_ASSERT_TRUE(e < n);
}

void test_throttle_drives_positive(void) {
    float t = longitudinal_compute({100.0f, 0.0f, 0.5f, DriveMode::Normal, true});
    TEST_ASSERT_TRUE(t > 0.0f);
}
void test_brake_regens_negative(void) {
    float t = longitudinal_compute({0.0f, 100.0f, 0.5f, DriveMode::Normal, true});
    TEST_ASSERT_TRUE(t < 0.0f);
}
void test_idle_is_zero(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, longitudinal_compute({0.0f, 0.0f, 0.5f, DriveMode::Normal, true}));
}
void test_regen_auto_off_blocks_regen(void) {
    TEST_ASSERT_EQUAL_FLOAT(
        0.0f,
        longitudinal_compute({0.0f, 100.0f, 0.5f, DriveMode::Normal, false}));
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_full_throttle_requests_configured_current_for_both_motors);
    RUN_TEST(test_throttle_drives_positive);
    RUN_TEST(test_brake_regens_negative);
    RUN_TEST(test_regen_auto_off_blocks_regen);
    RUN_TEST(test_idle_is_zero);
    RUN_TEST(test_regen_taper_midpoint);
    RUN_TEST(test_regen_cutoff_high_soc);
    RUN_TEST(test_throttle_wins_over_brake);
    RUN_TEST(test_release_qualification_and_immediate_cancel);
    RUN_TEST(test_invalid_inputs_do_not_request_regen);
    RUN_TEST(test_efficiency_limits_drive);
    return UNITY_END();
}

