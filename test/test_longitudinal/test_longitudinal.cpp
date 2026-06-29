#include <unity.h>
#include "modules/longitudinal.h"

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
    return UNITY_END();
}
