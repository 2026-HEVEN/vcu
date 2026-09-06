#include <unity.h>
#include "types.h"
#include "state.h"

void test_clamps_above_max(void) { Percent p = 150.0f; TEST_ASSERT_EQUAL_FLOAT(100.0f, (float)p); }
void test_clamps_below_min(void) { Percent p = -250.0f; TEST_ASSERT_EQUAL_FLOAT(-100.0f, (float)p); }
void test_passes_in_range(void) { Percent p = 42.0f; TEST_ASSERT_EQUAL_FLOAT(42.0f, (float)p); }
void test_assignment_clamps(void) { Unit u = 0.0f; u = 5.0f; TEST_ASSERT_EQUAL_FLOAT(1.0f, (float)u); }
void test_default_is_zero(void) { Rpm r; TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)r); }
void test_amp_clamps_above_max(void) { Amp a = 600.0f; TEST_ASSERT_EQUAL_FLOAT(500.0f, (float)a); }
void test_amp_clamps_below_min(void) { Amp a = -600.0f; TEST_ASSERT_EQUAL_FLOAT(-500.0f, (float)a); }
void test_amp_allows_continuous_limit(void) { Amp a = 103.0f; TEST_ASSERT_EQUAL_FLOAT(103.0f, (float)a); }
void test_amp_allows_peak_limit(void) { Amp a = 500.0f; TEST_ASSERT_EQUAL_FLOAT(500.0f, (float)a); }

void test_state_defaults_safe(void) {
    VehicleState s;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)s.throttle_pct);
    TEST_ASSERT_FALSE(s.throttle_signal_valid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)s.torque_L);
    TEST_ASSERT_FALSE(s.brake_active);
    TEST_ASSERT_FALSE(s.controller_feedback_fresh);
    TEST_ASSERT_TRUE(s.gear == Gear::Neutral);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_clamps_above_max);
    RUN_TEST(test_clamps_below_min);
    RUN_TEST(test_passes_in_range);
    RUN_TEST(test_amp_clamps_above_max);
    RUN_TEST(test_amp_clamps_below_min);
    RUN_TEST(test_amp_allows_continuous_limit);
    RUN_TEST(test_amp_allows_peak_limit);
    RUN_TEST(test_assignment_clamps);
    RUN_TEST(test_default_is_zero);
    RUN_TEST(test_state_defaults_safe);
    return UNITY_END();
}
