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

// Conditional integration must let the integral move OUT of saturation.
// With ki tied to the Mz limit only a derivative spike can hold the output
// saturated while the error has already reversed, so these use kd.
void test_release_from_high_saturation_reduces_integral() {
    TVParams p = active_params();
    p.kp = 2.0f; p.ki = 1.0f; p.kd = 1.0f;
    TVYawState s{};
    s.integral = 5.0f; s.initialized = true; s.prev_measured_yaw = 10.0f;
    // measured drops 10 -> 5: D is large and positive, error is negative
    tv_yaw_compute(0.0f, 5.0f, 0.01f, p, s);
    TEST_ASSERT_TRUE(s.integral < 5.0f);
}

void test_release_from_low_saturation_increases_integral() {
    TVParams p = active_params();
    p.kp = 2.0f; p.ki = 1.0f; p.kd = 1.0f;
    TVYawState s{};
    s.integral = -5.0f; s.initialized = true; s.prev_measured_yaw = -10.0f;
    tv_yaw_compute(0.0f, -5.0f, 0.01f, p, s);
    TEST_ASSERT_TRUE(s.integral > -5.0f);
}

void test_negative_saturation_does_not_wind_up() {
    TVYawState s{};
    TVParams p = active_params();
    for (int i = 0; i < 100; ++i) {
        const float mz = tv_yaw_compute(-100.0f, 0.0f, 0.01f, p, s);
        TEST_ASSERT_TRUE(std::fabs(mz) <= p.yaw_moment_max);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, s.integral);
}

void test_integral_limit_follows_ki_authority() {
    TVYawState s{};
    TVParams p = active_params();
    p.kp = 0.0f; p.ki = 2.0f; p.kd = 0.0f;
    p.integral_max = 100.0f; p.yaw_moment_max = 10.0f;  // ki*I <= 10 -> I <= 5
    for (int i = 0; i < 100; ++i) tv_yaw_compute(10.0f, 0.0f, 1.0f, p, s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, s.integral);
}

void test_held_sample_uses_real_interval() {
    TVYawState s{};
    TVParams p = active_params();
    p.kp = 0.0f; p.ki = 0.0f; p.kd = 1.0f;
    p.yaw_moment_max = 1000.0f; p.derivative_filter_tau_s = 0.0f;
    tv_yaw_compute(0.0f, 0.0f, 0.01f, p, s);                 // prime
    tv_yaw_compute(0.0f, 0.0f, 0.01f, p, s, false);          // same sample re-read
    const float out = tv_yaw_compute(0.0f, 2.0f, 0.01f, p, s, true);
    // 2 deg/s change over the real 20 ms sample interval, not over 10 ms
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -100.0f, out);
}

void test_derivative_filter_attenuates_a_step() {
    TVYawState s{};
    TVParams p = active_params();
    p.kp = 0.0f; p.ki = 0.0f; p.kd = 1.0f;
    p.yaw_moment_max = 1000.0f; p.derivative_filter_tau_s = 0.02f;
    tv_yaw_compute(0.0f, 0.0f, 0.01f, p, s);
    const float out = tv_yaw_compute(0.0f, 1.0f, 0.01f, p, s);
    // raw -100 deg/s^2 scaled by alpha = 0.01 / (0.02 + 0.01)
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -100.0f / 3.0f, out);
}

void test_negative_deadband_is_treated_as_zero() {
    TVYawState s{};
    TVParams p = active_params();
    p.kp = 1.0f; p.ki = 0.0f; p.kd = 0.0f;
    p.yaw_deadband_degps = -0.5f;
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.2f,
        tv_yaw_compute(0.0f, 0.2f, 0.01f, p, s));
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
    RUN_TEST(test_release_from_high_saturation_reduces_integral);
    RUN_TEST(test_release_from_low_saturation_increases_integral);
    RUN_TEST(test_negative_saturation_does_not_wind_up);
    RUN_TEST(test_integral_limit_follows_ki_authority);
    RUN_TEST(test_held_sample_uses_real_interval);
    RUN_TEST(test_derivative_filter_attenuates_a_step);
    RUN_TEST(test_negative_deadband_is_treated_as_zero);
    return UNITY_END();
}
