#include <unity.h>
#include "modules/steering.h"
#include "modules/realcar_calibration.h"
#include <cmath>

// center=8192, 4096 counts == 1.0 unit.
void test_centered(void)    { TEST_ASSERT_EQUAL_FLOAT(0.0f,  (float)steering_compute({8192}, {8192, 4096.0f, false})); }
void test_full_right(void)  { TEST_ASSERT_EQUAL_FLOAT(1.0f,  (float)steering_compute({12288},{8192, 4096.0f, false})); }
void test_full_left(void)   { TEST_ASSERT_EQUAL_FLOAT(-1.0f, (float)steering_compute({4096}, {8192, 4096.0f, false})); }
void test_invert(void)      { TEST_ASSERT_EQUAL_FLOAT(-1.0f, (float)steering_compute({12288},{8192, 4096.0f, true})); }
void test_clamps(void)      { TEST_ASSERT_EQUAL_FLOAT(1.0f,  (float)steering_compute({16383},{8192, 4096.0f, false})); }

// Sample validity shared by the dashboard and the TV gate.
void test_sample_invalid_when_sensor_not_installed(void) {
    TEST_ASSERT_FALSE(steering_sample_valid({8192}, 0.0f, false));
}
void test_sample_valid_inside_band(void) {
    TEST_ASSERT_TRUE(steering_sample_valid({8192}, 0.0f, true));
}
void test_sample_invalid_near_rails(void) {
    const uint16_t lo = (uint16_t)realcar_cal::bringup::STEERING_RAW_VALID_MIN_COUNTS;
    const uint16_t hi = (uint16_t)realcar_cal::bringup::STEERING_RAW_VALID_MAX_COUNTS;
    TEST_ASSERT_FALSE(steering_sample_valid({0}, -1.0f, true));      // open wiper
    TEST_ASSERT_FALSE(steering_sample_valid({16380}, 1.0f, true));   // shorted wiper
    TEST_ASSERT_TRUE(steering_sample_valid({lo}, -0.9f, true));
    TEST_ASSERT_FALSE(steering_sample_valid({(uint16_t)(lo - 4)}, -0.9f, true));
    TEST_ASSERT_TRUE(steering_sample_valid({hi}, 0.9f, true));
    TEST_ASSERT_FALSE(steering_sample_valid({(uint16_t)(hi + 4)}, 0.9f, true));
}
void test_sample_invalid_when_unit_not_finite(void) {
    TEST_ASSERT_FALSE(steering_sample_valid({8192}, NAN, true));
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_centered);
    RUN_TEST(test_full_right);
    RUN_TEST(test_full_left);
    RUN_TEST(test_invert);
    RUN_TEST(test_clamps);
    RUN_TEST(test_sample_invalid_when_sensor_not_installed);
    RUN_TEST(test_sample_valid_inside_band);
    RUN_TEST(test_sample_invalid_near_rails);
    RUN_TEST(test_sample_invalid_when_unit_not_finite);
    return UNITY_END();
}
