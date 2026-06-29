#include <unity.h>
#include "modules/throttle.h"

void test_zero_at_bottom(void)   { TEST_ASSERT_EQUAL_FLOAT(0.0f,   (float)throttle_compute({0})); }
void test_deadzone(void)         { TEST_ASSERT_EQUAL_FLOAT(0.0f,   (float)throttle_compute({150})); } // ~3.6% < 5%
void test_full_at_top(void)      { TEST_ASSERT_EQUAL_FLOAT(100.0f, (float)throttle_compute({4095})); }
void test_clamps_overrange(void) { TEST_ASSERT_EQUAL_FLOAT(100.0f, (float)throttle_compute({99999})); }

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_at_bottom);
    RUN_TEST(test_deadzone);
    RUN_TEST(test_full_at_top);
    RUN_TEST(test_clamps_overrange);
    return UNITY_END();
}
