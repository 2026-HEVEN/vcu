#include <unity.h>
#include "modules/imu.h"

void test_passes_through_yaw_rate(void) {
    ImuOutput o = imu_compute({10.0f, 0.0f, 0.0f});
    TEST_ASSERT_EQUAL_FLOAT(10.0f, o.yaw_rate);
}
void test_passes_through_accel(void) {
    ImuOutput o = imu_compute({0.0f, 0.3f, -0.2f});
    TEST_ASSERT_EQUAL_FLOAT(0.3f, o.accel_x);
    TEST_ASSERT_EQUAL_FLOAT(-0.2f, o.accel_y);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_passes_through_yaw_rate);
    RUN_TEST(test_passes_through_accel);
    return UNITY_END();
}
