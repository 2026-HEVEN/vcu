#include <unity.h>
#include "modules/steering.h"
#include "modules/realcar_calibration.h"

void test_measured_road_wheel_calibration() {
    using namespace realcar_cal::provisional;
    const SteerCalib c{STEERING_CENTER_COUNTS, STEERING_COUNTS_PER_UNIT, STEERING_INVERT};
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, (float)steering_compute({1700U * 4U}, c));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, (float)steering_compute({176U * 4U}, c));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -5096.0f/6060.0f, (float)steering_compute({2974U * 4U}, c));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 4096.0f/6060.0f, (float)steering_compute({676U * 4U}, c));
}

// center=8192, 4096 counts == 1.0 unit.
void test_centered(void)    { TEST_ASSERT_EQUAL_FLOAT(0.0f,  (float)steering_compute({8192}, {8192, 4096.0f, false})); }
void test_full_right(void)  { TEST_ASSERT_EQUAL_FLOAT(1.0f,  (float)steering_compute({12288},{8192, 4096.0f, false})); }
void test_full_left(void)   { TEST_ASSERT_EQUAL_FLOAT(-1.0f, (float)steering_compute({4096}, {8192, 4096.0f, false})); }
void test_invert(void)      { TEST_ASSERT_EQUAL_FLOAT(-1.0f, (float)steering_compute({12288},{8192, 4096.0f, true})); }
void test_clamps(void)      { TEST_ASSERT_EQUAL_FLOAT(1.0f,  (float)steering_compute({16383},{8192, 4096.0f, false})); }

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_measured_road_wheel_calibration);
    RUN_TEST(test_centered);
    RUN_TEST(test_full_right);
    RUN_TEST(test_full_left);
    RUN_TEST(test_invert);
    RUN_TEST(test_clamps);
    return UNITY_END();
}
