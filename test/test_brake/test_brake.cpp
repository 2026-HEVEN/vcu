#include <unity.h>
#include "modules/brake.h"

namespace {
const BrakeCalib CAL{
    400, 3400, 200, 3800, 100.0f,
    0.01f, 0.02f, 5.0f, 3.0f
};
const BrakeCalib NO_FILTER_CAL{
    400, 3400, 200, 3800, 100.0f,
    0.01f, 0.0f, 5.0f, 3.0f
};
}

void test_zero_pressure_is_valid_and_released(void) {
    BrakeFilterState state{};
    const BrakeOutput o = brake_compute({400}, CAL, state);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_FALSE(o.active);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)o.pct);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.pressure_bar);
}
void test_mid_scale_pressure_is_continuous(void) {
    BrakeFilterState state{};
    const BrakeOutput o = brake_compute({1900}, CAL, state);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_TRUE(o.active);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, (float)o.pct);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, o.pressure_bar);
}
void test_full_scale_clamps(void) {
    BrakeFilterState state{};
    const BrakeOutput o = brake_compute({3600}, CAL, state);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, (float)o.pct);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, o.pressure_bar);
}
void test_invalid_raw_fails_closed(void) {
    BrakeFilterState state{};
    const BrakeOutput low = brake_compute({199}, CAL, state);
    const BrakeOutput high = brake_compute({3801}, CAL, state);
    TEST_ASSERT_FALSE(low.valid);
    TEST_ASSERT_FALSE(high.valid);
    TEST_ASSERT_FALSE(low.active);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)low.pct);
}
void test_invalid_calibration_fails_closed(void) {
    const BrakeCalib zero_span{
        400, 400, 200, 3800, 100.0f,
        0.01f, 0.02f, 5.0f, 3.0f
    };
    BrakeFilterState state{};
    const BrakeOutput o = brake_compute({400}, zero_span, state);
    TEST_ASSERT_FALSE(o.valid);
    TEST_ASSERT_FALSE(o.active);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.pressure_bar);
}
void test_active_threshold_matches_five_percent_deadzone(void) {
    BrakeFilterState state{};
    TEST_ASSERT_FALSE(brake_compute({550}, NO_FILTER_CAL, state).active); // exactly 5%
    TEST_ASSERT_TRUE(brake_compute({551}, NO_FILTER_CAL, state).active);
}
void test_filter_smooths_adc_step(void) {
    BrakeFilterState state{};
    brake_compute({400}, CAL, state);
    const BrakeOutput o = brake_compute({1900}, CAL, state);
    // alpha = 0.01 / (0.02 + 0.01) = 1/3, so 400 -> 900.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 900.0f, o.filtered_adc);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 16.6667f, (float)o.pct);
}
void test_hysteresis_prevents_threshold_chatter(void) {
    BrakeFilterState state{};
    TEST_ASSERT_FALSE(brake_compute({550}, NO_FILTER_CAL, state).active); // 5%
    TEST_ASSERT_TRUE(brake_compute({551}, NO_FILTER_CAL, state).active);  // above 5%
    TEST_ASSERT_TRUE(brake_compute({520}, NO_FILTER_CAL, state).active);  // 4%, hold ON
    TEST_ASSERT_FALSE(brake_compute({490}, NO_FILTER_CAL, state).active); // 3%, turn OFF
}
void test_invalid_sample_resets_filter_and_active_state(void) {
    BrakeFilterState state{};
    TEST_ASSERT_TRUE(brake_compute({1900}, NO_FILTER_CAL, state).active);
    const BrakeOutput invalid = brake_compute({4095}, NO_FILTER_CAL, state);
    TEST_ASSERT_FALSE(invalid.valid);
    TEST_ASSERT_FALSE(invalid.active);
    TEST_ASSERT_FALSE(state.initialized);
    TEST_ASSERT_FALSE(state.active);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_pressure_is_valid_and_released);
    RUN_TEST(test_mid_scale_pressure_is_continuous);
    RUN_TEST(test_full_scale_clamps);
    RUN_TEST(test_invalid_raw_fails_closed);
    RUN_TEST(test_invalid_calibration_fails_closed);
    RUN_TEST(test_active_threshold_matches_five_percent_deadzone);
    RUN_TEST(test_filter_smooths_adc_step);
    RUN_TEST(test_hysteresis_prevents_threshold_chatter);
    RUN_TEST(test_invalid_sample_resets_filter_and_active_state);
    return UNITY_END();
}
