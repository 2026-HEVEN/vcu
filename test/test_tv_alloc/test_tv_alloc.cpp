#include <unity.h>
#include <cmath>
#include "modules/tv/allocation.h"

static MaxTorque unlimited() { return {1.0e6f, 1.0e6f}; }

void test_no_yaw_is_symmetric_and_preserves_total() {
    const TVAllocOutput o = tv_alloc_compute(20.0f, 0.0f, unlimited(), TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, (float)o.torque_L);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, (float)o.torque_R);
}

void test_positive_yaw_increases_right_current() {
    const TVAllocOutput o = tv_alloc_compute(40.0f, 10.0f, unlimited(), TV_PARAMS);
    TEST_ASSERT_TRUE((float)o.torque_R > (float)o.torque_L);
}

void test_drive_never_crosses_into_regen() {
    const TVAllocOutput o = tv_alloc_compute(5.0f, 1000.0f, unlimited(), TV_PARAMS);
    TEST_ASSERT_TRUE((float)o.torque_L >= 0.0f);
    TEST_ASSERT_TRUE((float)o.torque_R >= 0.0f);
}

void test_regen_never_crosses_into_drive() {
    const TVAllocOutput o = tv_alloc_compute(-60.0f, 1000.0f, unlimited(), TV_PARAMS);
    TEST_ASSERT_TRUE((float)o.torque_L <= 0.0f);
    TEST_ASSERT_TRUE((float)o.torque_R <= 0.0f);
}

void test_asymmetric_limits_are_respected() {
    const TVAllocOutput o = tv_alloc_compute(40.0f, 10.0f, {10.0f, 20.0f}, TV_PARAMS);
    TEST_ASSERT_TRUE((float)o.torque_L <= 10.0f);
    TEST_ASSERT_TRUE((float)o.torque_R <= 20.0f);
}

void test_zero_demand_cannot_create_torque() {
    const TVAllocOutput o = tv_alloc_compute(0.0f, 100.0f, unlimited(), TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, (float)o.torque_L);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, (float)o.torque_R);
}

void test_physical_yaw_to_current_conversion() {
    const TVAllocOutput o = tv_alloc_compute(40.0f, 10.0f, unlimited(), TV_PARAMS);
    const float expected_diff = 10.0f * TV_PARAMS.tire_radius_m /
        (TV_PARAMS.track_m * TV_PARAMS.motor_kt_nm_per_a * TV_PARAMS.gear_ratio);
    const float actual_diff = ((float)o.torque_R - (float)o.torque_L) * 0.5f;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_diff, actual_diff);
}

void test_bringup_limit_allows_300_a_but_no_more() {
    const TVAllocOutput o = tv_alloc_compute(1000.0f, 0.0f, unlimited(), TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 300.0f, (float)o.torque_L);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 300.0f, (float)o.torque_R);
}

void test_saturation_keeps_yaw_before_common_current() {
    TVParams p = TV_PARAMS;
    p.motor_current_max_a = 20.0f;
    const TVAllocOutput o = tv_alloc_compute(100.0f, 10.0f, {20.0f, 20.0f}, p);
    TEST_ASSERT_TRUE((float)o.torque_L + (float)o.torque_R < 100.0f);
    TEST_ASSERT_TRUE((float)o.torque_R > (float)o.torque_L);
}

// Regression: bug found 2026-08-25. total_current_a=1A, yaw_moment_nm=100Nm
// (well within Mz Max / TV_PARAMS.yaw_moment_max) used to produce
// L=0A, R=93A -- 92A of current the driver never asked for. No existing
// test caught this: test_drive_never_crosses_into_regen (above) uses this
// exact shape (small total, huge Mz) but only asserted non-negativity, not
// that the total was preserved.
void test_small_total_with_large_yaw_does_not_manufacture_current() {
    const TVAllocOutput o = tv_alloc_compute(1.0f, 100.0f, unlimited(), TV_PARAMS);
    const float sum = (float)o.torque_L + (float)o.torque_R;
    TEST_ASSERT_TRUE(sum <= 1.0f + 0.01f);
    TEST_ASSERT_TRUE(sum >= 0.0f);
}
void test_near_zero_total_with_large_yaw_does_not_manufacture_current() {
    const TVAllocOutput o = tv_alloc_compute(0.001f, 100.0f, unlimited(), TV_PARAMS);
    const float sum = (float)o.torque_L + (float)o.torque_R;
    TEST_ASSERT_TRUE(std::fabs(sum) <= 0.01f);
}
void test_regen_small_total_with_large_yaw_does_not_manufacture_current() {
    const TVAllocOutput o = tv_alloc_compute(-1.0f, 100.0f, unlimited(), TV_PARAMS);
    const float sum = (float)o.torque_L + (float)o.torque_R;
    TEST_ASSERT_TRUE(sum >= -1.0f - 0.01f);
    TEST_ASSERT_TRUE(sum <= 0.0f);
}
// Sweep: for any total/yaw combination, |L+R| must never exceed |total|,
// regardless of how large the yaw demand is relative to the total.
void test_sum_never_exceeds_total_across_a_sweep() {
    const float totals[] = {-90.0f, -50.0f, -1.0f, -0.001f, 0.001f, 1.0f, 5.0f, 50.0f, 90.0f};
    const float moments[] = {-1000.0f, -100.0f, -10.0f, 0.0f, 10.0f, 100.0f, 1000.0f};
    for (float total : totals) {
        for (float mz : moments) {
            const TVAllocOutput o = tv_alloc_compute(total, mz, unlimited(), TV_PARAMS);
            const float sum = (float)o.torque_L + (float)o.torque_R;
            TEST_ASSERT_TRUE(std::fabs(sum) <= std::fabs(total) + 0.05f);
        }
    }
}
// Preserved from before the fix: with ample total current relative to the
// yaw demand, the differential should still be applied undiminished.
void test_ample_total_still_gets_full_differential() {
    const TVAllocOutput o = tv_alloc_compute(50.0f, 20.0f, unlimited(), TV_PARAMS);
    const float expected_diff = 20.0f * TV_PARAMS.tire_radius_m /
        (TV_PARAMS.track_m * TV_PARAMS.motor_kt_nm_per_a * TV_PARAMS.gear_ratio);
    const float actual_diff = ((float)o.torque_R - (float)o.torque_L) * 0.5f;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_diff, actual_diff);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 50.0f, (float)o.torque_L + (float)o.torque_R);
}

void setUp() {}
void tearDown() {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_yaw_is_symmetric_and_preserves_total);
    RUN_TEST(test_positive_yaw_increases_right_current);
    RUN_TEST(test_drive_never_crosses_into_regen);
    RUN_TEST(test_regen_never_crosses_into_drive);
    RUN_TEST(test_asymmetric_limits_are_respected);
    RUN_TEST(test_zero_demand_cannot_create_torque);
    RUN_TEST(test_physical_yaw_to_current_conversion);
    RUN_TEST(test_bringup_limit_allows_300_a_but_no_more);
    RUN_TEST(test_saturation_keeps_yaw_before_common_current);
    RUN_TEST(test_small_total_with_large_yaw_does_not_manufacture_current);
    RUN_TEST(test_near_zero_total_with_large_yaw_does_not_manufacture_current);
    RUN_TEST(test_regen_small_total_with_large_yaw_does_not_manufacture_current);
    RUN_TEST(test_sum_never_exceeds_total_across_a_sweep);
    RUN_TEST(test_ample_total_still_gets_full_differential);
    return UNITY_END();
}
