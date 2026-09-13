#include <unity.h>
#include "modules/longitudinal.h"
#include "modules/realcar_calibration.h"

void test_full_throttle_requests_configured_current_for_both_motors(void) {
    const float total = longitudinal_compute(
        {100.0f, 0.0f, 0.5f, DriveMode::Normal, true, false});
    TEST_ASSERT_FLOAT_WITHIN(
        0.01f,
        2.0f * realcar_cal::bringup::DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A,
        total);
}

void test_regen_taper_midpoint(void) {
    float t = longitudinal_compute({0.0f, 100.0f, 0.925f, DriveMode::Normal, true, true});
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -10.0f, t);
}

void test_regen_cutoff_high_soc(void) {
    TEST_ASSERT_EQUAL_FLOAT(
        0.0f,
        longitudinal_compute({0.0f, 100.0f, 0.96f, DriveMode::Normal, true, true})
    );
}

void test_brake_override_kills_drive(void) {
    float t = longitudinal_compute(
        {100.0f, 10.0f, 0.5f, DriveMode::Normal, true, true});

    TEST_ASSERT_TRUE(t <= 0.0f);
}

void test_efficiency_limits_drive(void) {
    float e =
        longitudinal_compute(
            {100.0f,0.0f,0.5f,DriveMode::Efficiency, true, false});

    float n =
        longitudinal_compute(
            {100.0f,0.0f,0.5f,DriveMode::Normal, true, false});

    TEST_ASSERT_TRUE(e < n);
}

void test_throttle_drives_positive(void) {
    float t = longitudinal_compute({100.0f, 0.0f, 0.5f, DriveMode::Normal, true, false});
    TEST_ASSERT_TRUE(t > 0.0f);
}
void test_brake_regens_negative(void) {
    float t = longitudinal_compute({0.0f, 100.0f, 0.5f, DriveMode::Normal, true, true});
    TEST_ASSERT_TRUE(t < 0.0f);
}
void test_idle_is_zero(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, longitudinal_compute({0.0f, 0.0f, 0.5f, DriveMode::Normal, true, false}));
}
void test_regen_auto_off_blocks_regen(void) {
    TEST_ASSERT_EQUAL_FLOAT(
        0.0f,
        longitudinal_compute({0.0f, 100.0f, 0.5f, DriveMode::Normal, false, true}));
}
void test_regen_deadzone_is_zero_through_five_percent(void) {
    TEST_ASSERT_EQUAL_FLOAT(
        0.0f,
        longitudinal_compute({0.0f, 5.0f, 0.5f, DriveMode::Normal, true, false}));
}
void test_regen_ramps_continuously_above_deadzone(void) {
    const float t = longitudinal_compute(
        {0.0f, 5.95f, 0.5f, DriveMode::Normal, true, true});
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.2f, t);
}
void test_hysteretic_active_state_keeps_drive_overridden_on_release(void) {
    const float t = longitudinal_compute(
        {100.0f, 4.0f, 0.5f, DriveMode::Normal, true, true});
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t);
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
    RUN_TEST(test_brake_override_kills_drive);
    RUN_TEST(test_efficiency_limits_drive);
    RUN_TEST(test_regen_deadzone_is_zero_through_five_percent);
    RUN_TEST(test_regen_ramps_continuously_above_deadzone);
    RUN_TEST(test_hysteretic_active_state_keeps_drive_overridden_on_release);
    return UNITY_END();
}

