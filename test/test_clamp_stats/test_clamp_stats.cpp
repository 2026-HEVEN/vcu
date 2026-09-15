// 포화 진단: Clamped가 값을 자를 때 무엇이 어디서 잘렸는지 기록하는지.
// 클램프는 지금까지 완전히 무음이었다 — 주행 중 Amp가 500에 몇 번 걸렸는지
// 아무도 알 수 없었다. 기록은 set()에서, 출력은 debug_monitor가 한다.
#include <unity.h>
#include <cstring>
#include "types.h"

void setUp(void) {
    Amp::reset_clamp_stats();
    Percent::reset_clamp_stats();
}
void tearDown(void) {}

void test_in_range_records_nothing(void) {
    Amp a{250.0f};
    TEST_ASSERT_EQUAL_FLOAT(250.0f, (float)a);
    const ClampStats s = Amp::clamp_stats();
    TEST_ASSERT_EQUAL_UINT32(0, s.high_count);
    TEST_ASSERT_EQUAL_UINT32(0, s.low_count);
}

void test_upper_bound_is_counted_separately_from_lower(void) {
    Amp hi{600.0f};
    TEST_ASSERT_EQUAL_FLOAT(500.0f, (float)hi);
    const ClampStats s = Amp::clamp_stats();
    TEST_ASSERT_EQUAL_UINT32(1, s.high_count);
    TEST_ASSERT_EQUAL_UINT32(0, s.low_count);
}

void test_lower_bound_is_counted_separately_from_upper(void) {
    Amp lo{-600.0f};
    TEST_ASSERT_EQUAL_FLOAT(-500.0f, (float)lo);
    const ClampStats s = Amp::clamp_stats();
    TEST_ASSERT_EQUAL_UINT32(0, s.high_count);
    TEST_ASSERT_EQUAL_UINT32(1, s.low_count);
}

void test_raw_value_before_clamping_is_kept(void) {
    Amp a{812.4f};
    const ClampStats s = Amp::clamp_stats();
    TEST_ASSERT_EQUAL_FLOAT(812.4f, s.high_worst.raw);
}

void test_worst_is_kept_not_the_most_recent(void) {
    // 튜닝에 필요한 건 "가장 크게 빗나간 요구"다. 마지막 값으로 덮어쓰면
    // 최악의 순간이 그 다음 평범한 클램프에 지워진다.
    Amp big{900.0f};
    Amp small{510.0f};
    const ClampStats s = Amp::clamp_stats();
    TEST_ASSERT_EQUAL_UINT32(2, s.high_count);
    TEST_ASSERT_EQUAL_FLOAT(900.0f, s.high_worst.raw);
    TEST_ASSERT_EQUAL_FLOAT(510.0f, s.last.raw);   // 최근 값도 따로 남는다
}

void test_low_worst_keeps_the_most_negative(void) {
    Amp a{-700.0f};
    Amp b{-505.0f};
    const ClampStats s = Amp::clamp_stats();
    TEST_ASSERT_EQUAL_FLOAT(-700.0f, s.low_worst.raw);
}

void test_call_site_is_captured(void) {
    const int expected_line = __LINE__ + 1;
    Amp a{777.0f};
    const ClampStats s = Amp::clamp_stats();
    TEST_ASSERT_EQUAL_INT(expected_line, s.high_worst.line);
    TEST_ASSERT_NOT_NULL(s.high_worst.file);
    TEST_ASSERT_NOT_NULL(s.high_worst.fn);
    TEST_ASSERT_TRUE(std::strstr(s.high_worst.file, "test_clamp_stats") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(s.high_worst.fn, "test_call_site_is_captured") != nullptr);
}

void test_each_domain_type_counts_independently(void) {
    Amp a{600.0f};
    TEST_ASSERT_EQUAL_UINT32(1, Amp::clamp_stats().high_count);
    TEST_ASSERT_EQUAL_UINT32(0, Percent::clamp_stats().high_count);
    Percent p{150.0f};
    TEST_ASSERT_EQUAL_UINT32(1, Percent::clamp_stats().high_count);
    TEST_ASSERT_EQUAL_UINT32(1, Amp::clamp_stats().high_count);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_in_range_records_nothing);
    RUN_TEST(test_upper_bound_is_counted_separately_from_lower);
    RUN_TEST(test_lower_bound_is_counted_separately_from_upper);
    RUN_TEST(test_raw_value_before_clamping_is_kept);
    RUN_TEST(test_worst_is_kept_not_the_most_recent);
    RUN_TEST(test_low_worst_keeps_the_most_negative);
    RUN_TEST(test_call_site_is_captured);
    RUN_TEST(test_each_domain_type_counts_independently);
    return UNITY_END();
}
