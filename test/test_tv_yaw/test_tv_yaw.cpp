#include <unity.h>
#include "modules/tv/yaw_control.h"
#include <cmath>

static TVParams active_params() {
    TVParams p = TV_PARAMS;
    p.kp = 2.0f; p.ki = 1.0f; p.kd = 0.1f;
    p.yaw_deadband_degps = 0.0f;
    p.yaw_moment_max = 20.0f;
    return p;
}

void test_zero_error_zero_moment() {
    TVYawState s{};
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,
        tv_yaw_compute(10.0f, 10.0f, 0.01f, active_params(), s));
}

void test_positive_error_produces_positive_moment() {
    TVYawState s{};
    TEST_ASSERT_TRUE(tv_yaw_compute(10.0f, 0.0f, 0.01f,
                                    active_params(), s) > 0.0f);
}

void test_output_saturates_without_windup() {
    TVYawState s{};
    TVParams p = active_params();
    for (int i = 0; i < 100; ++i) {
        const float mz = tv_yaw_compute(100.0f, 0.0f, 0.01f, p, s);
        TEST_ASSERT_TRUE(std::fabs(mz) <= p.yaw_moment_max);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, s.integral);
}

void test_invalid_dt_is_safe() {
    TVYawState s{};
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,
        tv_yaw_compute(10.0f, 0.0f, 0.0f, active_params(), s));
}

void test_integral_has_hard_limit() {
    TVYawState s{};
    TVParams p = active_params();
    p.kp = 0.0f; p.ki = 0.01f; p.kd = 0.0f;
    p.integral_max = 2.0f;
    p.yaw_moment_max = 100.0f;
    for (int i = 0; i < 100; ++i) tv_yaw_compute(10.0f, 0.0f, 1.0f, p, s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, s.integral);
}

void test_deadband_is_continuous() {
    TVYawState a{}, b{};
    TVParams p = active_params();
    p.kp = 1.0f; p.ki = 0.0f; p.kd = 0.0f;
    p.yaw_deadband_degps = 0.5f;
    const float inside = tv_yaw_compute(0.49f, 0.0f, 0.01f, p, a);
    const float just_outside = tv_yaw_compute(0.51f, 0.0f, 0.01f, p, b);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, inside);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.01f, just_outside);
}

void test_target_step_does_not_create_derivative_kick() {
    TVYawState s{};
    TVParams p = active_params();
    p.kp = 0.0f; p.ki = 0.0f; p.kd = 1.0f;
    tv_yaw_compute(0.0f, 0.0f, 0.01f, p, s);
    const float output = tv_yaw_compute(20.0f, 0.0f, 0.01f, p, s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, output);
}

void setUp() {}
void tearDown() {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_error_zero_moment);
    RUN_TEST(test_positive_error_produces_positive_moment);
    RUN_TEST(test_output_saturates_without_windup);
    RUN_TEST(test_invalid_dt_is_safe);
    RUN_TEST(test_integral_has_hard_limit);
    RUN_TEST(test_deadband_is_continuous);
    RUN_TEST(test_target_step_does_not_create_derivative_kick);
    return UNITY_END();
}
