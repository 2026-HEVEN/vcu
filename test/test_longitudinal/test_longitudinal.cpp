#include <unity.h>
#include "modules/longitudinal.h"

void test_regen_taper_midpoint(void) {
    float t = longitudinal_compute({0.0f, 100.0f, 0.925f, DriveMode::Normal});
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -10.0f, t);
}

void test_regen_cutoff_high_soc(void) {
    TEST_ASSERT_EQUAL_FLOAT(
        0.0f,
        longitudinal_compute({0.0f, 100.0f, 0.96f, DriveMode::Normal})
    );
}

void test_brake_override_kills_drive(void) {
    float t = longitudinal_compute(
        {100.0f, 10.0f, 0.5f, DriveMode::Normal});

    TEST_ASSERT_TRUE(t <= 0.0f);
}

void test_efficiency_limits_drive(void) {
    float e =
        longitudinal_compute(
            {100.0f,0.0f,0.5f,DriveMode::Efficiency});

    float n =
        longitudinal_compute(
            {100.0f,0.0f,0.5f,DriveMode::Normal});

    TEST_ASSERT_TRUE(e < n);
}

void test_throttle_drives_positive(void) {
    float t = longitudinal_compute({100.0f, 0.0f, 0.5f, DriveMode::Normal});
    TEST_ASSERT_TRUE(t > 0.0f);
}
void test_brake_regens_negative(void) {
    float t = longitudinal_compute({0.0f, 100.0f, 0.5f, DriveMode::Normal});
    TEST_ASSERT_TRUE(t < 0.0f);
}
void test_idle_is_zero(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, longitudinal_compute({0.0f, 0.0f, 0.5f, DriveMode::Normal}));
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_throttle_drives_positive);
    RUN_TEST(test_brake_regens_negative);
    RUN_TEST(test_idle_is_zero);
    RUN_TEST(test_regen_taper_midpoint);
    RUN_TEST(test_regen_cutoff_high_soc);
    RUN_TEST(test_brake_override_kills_drive);
    RUN_TEST(test_efficiency_limits_drive);
    return UNITY_END();
}

