#include <unity.h>
#include "modules/tv/allocation.h"

static MaxTorque unlimited() { return {100.0f, 100.0f}; }

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
    return UNITY_END();
}
