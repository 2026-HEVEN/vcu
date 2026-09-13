#include <unity.h>
#include "modules/brake.h"

namespace {
const BrakeCalib CAL{400, 3400, 200, 3800, 100.0f, 5.0f};
}

void test_zero_pressure_is_valid_and_released(void) {
    const BrakeOutput o = brake_compute({400}, CAL);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_FALSE(o.active);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)o.pct);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.pressure_bar);
}
void test_mid_scale_pressure_is_continuous(void) {
    const BrakeOutput o = brake_compute({1900}, CAL);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_TRUE(o.active);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, (float)o.pct);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, o.pressure_bar);
}
void test_full_scale_clamps(void) {
    const BrakeOutput o = brake_compute({3600}, CAL);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, (float)o.pct);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, o.pressure_bar);
}
void test_invalid_raw_fails_closed(void) {
    const BrakeOutput low = brake_compute({199}, CAL);
    const BrakeOutput high = brake_compute({3801}, CAL);
    TEST_ASSERT_FALSE(low.valid);
    TEST_ASSERT_FALSE(high.valid);
    TEST_ASSERT_FALSE(low.active);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)low.pct);
}
void test_invalid_calibration_fails_closed(void) {
    const BrakeCalib zero_span{400, 400, 200, 3800, 100.0f, 5.0f};
    const BrakeOutput o = brake_compute({400}, zero_span);
    TEST_ASSERT_FALSE(o.valid);
    TEST_ASSERT_FALSE(o.active);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, o.pressure_bar);
}
void test_active_threshold_matches_five_percent_deadzone(void) {
    TEST_ASSERT_FALSE(brake_compute({550}, CAL).active); // exactly 5%
    TEST_ASSERT_TRUE(brake_compute({551}, CAL).active);
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
    return UNITY_END();
}
