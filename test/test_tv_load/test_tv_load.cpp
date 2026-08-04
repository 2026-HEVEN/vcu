#include <unity.h>
#include "modules/tv/load.h"

void test_static_load_is_symmetric() {
    const WheelLoads fz = tv_load_compute(0.0f, 0.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, fz.fz_L, fz.fz_R);
    TEST_ASSERT_TRUE(fz.fz_L > 0.0f);
}

void test_positive_lateral_g_loads_right_wheel() {
    const WheelLoads fz = tv_load_compute(0.0f, 0.5f, TV_PARAMS);
    TEST_ASSERT_TRUE(fz.fz_R > fz.fz_L);
}

void test_forward_acceleration_increases_rear_load() {
    const WheelLoads steady = tv_load_compute(0.0f, 0.0f, TV_PARAMS);
    const WheelLoads accel = tv_load_compute(0.5f, 0.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(accel.fz_L + accel.fz_R > steady.fz_L + steady.fz_R);
}

void test_extreme_input_never_returns_negative_load() {
    const WheelLoads fz = tv_load_compute(-10.0f, 10.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(fz.fz_L >= 0.0f && fz.fz_R >= 0.0f);
}

void test_lateral_transfer_conserves_rear_axle_sum() {
    const WheelLoads steady = tv_load_compute(0.0f, 0.0f, TV_PARAMS);
    const WheelLoads corner = tv_load_compute(0.0f, 0.7f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, steady.fz_L + steady.fz_R,
                            corner.fz_L + corner.fz_R);
}

void test_lltd_is_independent_from_static_distribution() {
    TVParams low = TV_PARAMS, high = TV_PARAMS;
    low.lltd_r = 0.2f; high.lltd_r = 0.8f;
    const WheelLoads a = tv_load_compute(0.0f, 0.5f, low);
    const WheelLoads b = tv_load_compute(0.0f, 0.5f, high);
    TEST_ASSERT_TRUE((b.fz_R - b.fz_L) > (a.fz_R - a.fz_L));
}

void setUp() {}
void tearDown() {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_static_load_is_symmetric);
    RUN_TEST(test_positive_lateral_g_loads_right_wheel);
    RUN_TEST(test_forward_acceleration_increases_rear_load);
    RUN_TEST(test_extreme_input_never_returns_negative_load);
    RUN_TEST(test_lateral_transfer_conserves_rear_axle_sum);
    RUN_TEST(test_lltd_is_independent_from_static_distribution);
    return UNITY_END();
}
