#include <unity.h>
#include "modules/torque_vectoring.h"

void test_straight_is_symmetric(void) {
    TVOutput o = tv_compute({20.0f, 0.0f, 0.0f, 0.0f});
    TEST_ASSERT_FLOAT_WITHIN(0.01f, (float)o.torque_L, (float)o.torque_R);
}
void test_split_sums_to_demand(void) {
    TVOutput o = tv_compute({20.0f, 0.0f, 0.0f, 0.0f});
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, (float)o.torque_L + (float)o.torque_R);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_straight_is_symmetric);
    RUN_TEST(test_split_sums_to_demand);
    return UNITY_END();
}
