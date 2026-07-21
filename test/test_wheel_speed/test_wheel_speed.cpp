#include <unity.h>
#include "modules/wheel_speed.h"

// 45 pulses/rev. 45 pulses in 100ms => 10 rev/s => 600 rpm.
void test_typical(void) {
    Rpm r = wheel_speed_compute({45, 100}, {45.0f});
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 600.0f, (float)r);
}
void test_zero_dt_safe(void) {  // never divide by zero
    Rpm r = wheel_speed_compute({10, 0}, {45.0f});
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)r);
}
// 45 pulses in 200ms => 5 rev/s => 300 rpm
void test_half_speed(void) {
    Rpm r = wheel_speed_compute({45, 200}, {45.0f});
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 300.0f, (float)r);
}
// 말도 안 되는 과대 입력은 Rpm(0..6000)이 상한으로 막는다
void test_clamps_high(void) {
    Rpm r = wheel_speed_compute({100000, 100}, {45.0f});
    TEST_ASSERT_EQUAL_FLOAT(6000.0f, (float)r);
}
// 보정상수 0/음수는 안전하게 0
void test_bad_calib_safe(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)wheel_speed_compute({45, 100}, {0.0f}));
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_typical);
    RUN_TEST(test_zero_dt_safe);
    RUN_TEST(test_half_speed);
    RUN_TEST(test_clamps_high);
    RUN_TEST(test_bad_calib_safe);
    return UNITY_END();
}
