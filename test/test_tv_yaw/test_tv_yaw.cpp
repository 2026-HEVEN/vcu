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

void setUp() {}
void tearDown() {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_error_zero_moment);
    RUN_TEST(test_positive_error_produces_positive_moment);
    RUN_TEST(test_output_saturates_without_windup);
    RUN_TEST(test_invalid_dt_is_safe);
    return UNITY_END();
}
