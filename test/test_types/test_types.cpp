#include <unity.h>
#include "types.h"

void test_clamps_above_max(void) { Percent p = 150.0f; TEST_ASSERT_EQUAL_FLOAT(100.0f, (float)p); }
void test_clamps_below_min(void) { Percent p = -250.0f; TEST_ASSERT_EQUAL_FLOAT(-100.0f, (float)p); }
void test_passes_in_range(void) { Percent p = 42.0f; TEST_ASSERT_EQUAL_FLOAT(42.0f, (float)p); }
void test_assignment_clamps(void) { Unit u = 0.0f; u = 5.0f; TEST_ASSERT_EQUAL_FLOAT(1.0f, (float)u); }
void test_default_is_zero(void) { Rpm r; TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)r); }

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_clamps_above_max);
    RUN_TEST(test_clamps_below_min);
    RUN_TEST(test_passes_in_range);
    RUN_TEST(test_assignment_clamps);
    RUN_TEST(test_default_is_zero);
    return UNITY_END();
}
