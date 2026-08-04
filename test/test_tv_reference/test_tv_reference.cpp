#include <unity.h>
#include "modules/tv/reference.h"
#include <cmath>

void test_zero_and_low_speed_are_disabled() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,
        tv_reference_compute(Unit(0.5f), 0.5f, TV_PARAMS));
}

void test_steering_sign_and_symmetry() {
    const float left = tv_reference_compute(Unit(0.25f), 5.0f, TV_PARAMS);
    const float right = tv_reference_compute(Unit(-0.25f), 5.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(left > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, left, -right);
}

void test_target_is_limited() {
    const float yaw = tv_reference_compute(Unit(1.0f), 30.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(std::fabs(yaw) <= TV_PARAMS.desired_yaw_max);
}

void setUp() {}
void tearDown() {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_and_low_speed_are_disabled);
    RUN_TEST(test_steering_sign_and_symmetry);
    RUN_TEST(test_target_is_limited);
    return UNITY_END();
}
