#include <unity.h>
#include "modules/tv/traction.h"

void test_equal_load_equal_limit() {
    const MaxTorque m = tv_traction_compute({800.0f, 800.0f}, 0.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, m.max_L, m.max_R);
    TEST_ASSERT_TRUE(m.max_L > 0.0f);
}

void test_limit_never_exceeds_motor_current_limit() {
    const MaxTorque m = tv_traction_compute({10000.0f, 10000.0f}, 0.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(m.max_L <= TV_PARAMS.motor_current_max_a);
    TEST_ASSERT_TRUE(m.max_R <= TV_PARAMS.motor_current_max_a);
}

void test_more_lateral_accel_reduces_current_capacity() {
    TVParams p = TV_PARAMS;
    p.motor_current_max_a = 1000.0f;
    const MaxTorque straight = tv_traction_compute({800.0f, 800.0f}, 0.0f, p);
    const MaxTorque corner = tv_traction_compute({800.0f, 800.0f}, 0.5f, p);
    TEST_ASSERT_TRUE(corner.max_L < straight.max_L);
}

void test_zero_load_is_safe() {
    const MaxTorque m = tv_traction_compute({0.0f, 0.0f}, 1.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, m.max_L);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, m.max_R);
}

void setUp() {}
void tearDown() {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_equal_load_equal_limit);
    RUN_TEST(test_limit_never_exceeds_motor_current_limit);
    RUN_TEST(test_more_lateral_accel_reduces_current_capacity);
    RUN_TEST(test_zero_load_is_safe);
    return UNITY_END();
}
