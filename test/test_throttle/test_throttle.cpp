#include <unity.h>
#include "modules/throttle.h"

void test_zero_at_bottom(void)   { TEST_ASSERT_EQUAL_FLOAT(0.0f,   (float)throttle_compute({0})); }
void test_deadzone(void)         { TEST_ASSERT_EQUAL_FLOAT(0.0f,   (float)throttle_compute({150})); } // RAW_MIN 아래
void test_zero_at_calibrated_min(void) { TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)throttle_compute({500})); }
void test_positive_above_calibrated_min(void) { TEST_ASSERT_TRUE((float)throttle_compute({501}) > 0.0f); }
void test_full_at_calibrated_max(void) { TEST_ASSERT_EQUAL_FLOAT(100.0f, (float)throttle_compute({3000})); }
void test_full_at_top(void)      { TEST_ASSERT_EQUAL_FLOAT(100.0f, (float)throttle_compute({4095})); }
void test_clamps_overrange(void) { TEST_ASSERT_EQUAL_FLOAT(100.0f, (float)throttle_compute({99999})); }

// 중간 입력은 0~100% 사이의 부분값 + 단조 증가
void test_partial_in_middle(void) {
    float mid = throttle_compute({2000});
    TEST_ASSERT_TRUE(mid > 0.0f && mid < 100.0f);
}
void test_monotonic(void) {
    TEST_ASSERT_TRUE((float)throttle_compute({2500}) > (float)throttle_compute({1500}));
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_at_bottom);
    RUN_TEST(test_deadzone);
    RUN_TEST(test_zero_at_calibrated_min);
    RUN_TEST(test_positive_above_calibrated_min);
    RUN_TEST(test_full_at_calibrated_max);
    RUN_TEST(test_full_at_top);
    RUN_TEST(test_clamps_overrange);
    RUN_TEST(test_partial_in_middle);
    RUN_TEST(test_monotonic);
    return UNITY_END();
}
