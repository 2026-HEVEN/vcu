#include <unity.h>
#include "modules/wheel_speed.h"
#include "modules/realcar_calibration.h"

static constexpr float PPR = realcar_cal::confirmed::WSS_PULSES_PER_WHEEL_REV_FL;

// 24 pulses/rev. 24 pulses in 100ms => 10 rev/s => 600 rpm.
void test_typical(void) {
    Rpm r = wheel_speed_compute({24, 100}, {PPR});
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 600.0f, (float)r);
}
void test_zero_dt_safe(void) {  // never divide by zero
    Rpm r = wheel_speed_compute({10, 0}, {PPR});
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)r);
}
// 24 pulses in 200ms => 5 rev/s => 300 rpm
void test_half_speed(void) {
    Rpm r = wheel_speed_compute({24, 200}, {PPR});
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 300.0f, (float)r);
}
// 말도 안 되는 과대 입력은 Rpm(0..6000)이 상한으로 막는다
void test_clamps_high(void) {
    Rpm r = wheel_speed_compute({100000, 100}, {PPR});
    TEST_ASSERT_EQUAL_FLOAT(6000.0f, (float)r);
}
// 보정상수 0/음수는 안전하게 0
void test_bad_calib_safe(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)wheel_speed_compute({48, 100}, {0.0f}));
}

void test_realcar_ppr_is_24_rising_edges(void) {
    TEST_ASSERT_EQUAL_FLOAT(24.0f, PPR);
}

void test_filter_suppresses_one_pulse_10ms_quantization(void) {
    WssCalib c{PPR, realcar_cal::provisional::WSS_FILTER_TIME_CONSTANT_S};
    WheelSpeedFilterState s{};
    const float first = (float)wheel_speed_compute_filtered({4, 10}, c, s);
    const float next  = (float)wheel_speed_compute_filtered({5, 10}, c, s);

    // 24 PPR/10 ms에서 원시 1 pulse 차이는 250 rpm이지만 필터 출력은
    // 100 Hz 차속 타당성 검사를 통과할 정도로 완만해야 한다.
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1000.0f, first);
    TEST_ASSERT_TRUE((next - first) < 11.0f);
    TEST_ASSERT_TRUE(next > first);
}

void test_filter_holds_last_value_on_zero_dt(void) {
    WssCalib c{PPR, realcar_cal::provisional::WSS_FILTER_TIME_CONSTANT_S};
    WheelSpeedFilterState s{};
    const float first = (float)wheel_speed_compute_filtered({4, 10}, c, s);
    const float held  = (float)wheel_speed_compute_filtered({99, 0}, c, s);
    TEST_ASSERT_EQUAL_FLOAT(first, held);
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
    RUN_TEST(test_realcar_ppr_is_24_rising_edges);
    RUN_TEST(test_filter_suppresses_one_pulse_10ms_quantization);
    RUN_TEST(test_filter_holds_last_value_on_zero_dt);
    return UNITY_END();
}
