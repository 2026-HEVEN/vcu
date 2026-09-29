#include <unity.h>
#include "modules/imu.h"

void test_passes_through_yaw_rate(void) {
    ImuOutput o = imu_compute({10.0f, 0.0f, 0.0f});
    TEST_ASSERT_EQUAL_FLOAT(10.0f, o.yaw_rate);
}
void test_reverses_both_accel_axes(void) {
    ImuOutput o = imu_compute({0.0f, 0.3f, -0.2f});
    TEST_ASSERT_EQUAL_FLOAT(-0.3f, o.accel_x);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, o.accel_y);
}
void test_vehicle_acceleration_and_yaw_signs(void) {
    ImuOutput o = imu_compute({-12.0f, -0.4f, 0.5f});
    TEST_ASSERT_EQUAL_FLOAT(-12.0f, o.yaw_rate);
    TEST_ASSERT_EQUAL_FLOAT(0.4f, o.accel_x);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, o.accel_y);
}
void test_zero_acceleration_stays_zero(void) {
    ImuOutput o = imu_compute({0.0f, 0.0f, 0.0f});
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.yaw_rate);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.accel_x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.accel_y);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_passes_through_yaw_rate);
    RUN_TEST(test_reverses_both_accel_axes);
    RUN_TEST(test_vehicle_acceleration_and_yaw_signs);
    RUN_TEST(test_zero_acceleration_stays_zero);
    return UNITY_END();
}
