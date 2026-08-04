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

void setUp() {}
void tearDown() {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_static_load_is_symmetric);
    RUN_TEST(test_positive_lateral_g_loads_right_wheel);
    RUN_TEST(test_forward_acceleration_increases_rear_load);
    RUN_TEST(test_extreme_input_never_returns_negative_load);
    return UNITY_END();
}
