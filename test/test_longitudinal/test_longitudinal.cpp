#include <unity.h>
#include "modules/longitudinal.h"

namespace {
float run(LongInput in, LongitudinalState &state, float seconds = 1.0f) {
    constexpr float DT_S = 0.01f;
    const int steps = static_cast<int>(seconds / DT_S);
    float output = 0.0f;
    for (int i = 0; i < steps; ++i) {
        output = longitudinal_compute(in, state, DT_S);
    }
    return output;
}
}

void test_throttle_drives_positive(void) {
    LongitudinalState state{};
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f,
        run({100.0f, 0.0f, 0.5f, 1000.0f, DriveMode::Normal}, state));
}

void test_brake_override_kills_drive(void) {
    LongitudinalState state{};
    const float output = run(
        {100.0f, 100.0f, 0.5f, 1000.0f, DriveMode::Normal}, state);
    TEST_ASSERT_TRUE(output < 0.0f);
}

void test_idle_is_zero(void) {
    LongitudinalState state{};
    TEST_ASSERT_EQUAL_FLOAT(0.0f,
        run({0.0f, 0.0f, 0.5f, 1000.0f, DriveMode::Normal}, state));
}

void test_regen_cuts_out_at_80_rpm(void) {
    LongitudinalState state{};
    TEST_ASSERT_EQUAL_FLOAT(0.0f,
        run({0.0f, 100.0f, 0.5f, 80.0f, DriveMode::Normal}, state));
}

void test_regen_smoothstep_at_300_rpm(void) {
    LongitudinalState state{};
    const float output = run(
        {0.0f, 100.0f, 0.5f, 300.0f, DriveMode::Normal}, state);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, -32.1f, output);
}

void test_regen_reaches_60a_mid_speed(void) {
    LongitudinalState state{};
    const float output = run(
        {0.0f, 100.0f, 0.5f, 1000.0f, DriveMode::Normal}, state);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -60.0f, output);
}

void test_regen_uses_inverse_rpm_power_limit(void) {
    LongitudinalState state{};
    const float output = run(
        {0.0f, 100.0f, 0.5f, 4000.0f, DriveMode::Normal}, state);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -32.475f, output);
}

void test_regen_taper_midpoint(void) {
    LongitudinalState state{};
    const float output = run(
        {0.0f, 100.0f, 0.925f, 1000.0f, DriveMode::Normal}, state);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -30.0f, output);
}

void test_regen_cutoff_high_soc_is_immediate(void) {
    LongitudinalState state{};
    run({0.0f, 100.0f, 0.5f, 1000.0f, DriveMode::Normal}, state);
    TEST_ASSERT_EQUAL_FLOAT(0.0f,
        longitudinal_compute(
            {0.0f, 100.0f, 0.96f, 1000.0f, DriveMode::Normal}, state, 0.01f));
}

void test_regen_rise_rate_is_10a_per_100ms(void) {
    LongitudinalState state{};
    const float output = run(
        {0.0f, 100.0f, 0.5f, 1000.0f, DriveMode::Normal}, state, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -10.0f, output);
}

void test_regen_fall_rate_is_20a_per_100ms(void) {
    LongitudinalState state{};
    run({0.0f, 100.0f, 0.5f, 1000.0f, DriveMode::Normal}, state);
    const float output = run(
        {0.0f, 0.0f, 0.5f, 1000.0f, DriveMode::Normal}, state, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -40.0f, output);
}

void test_efficiency_limits_drive(void) {
    LongitudinalState normal_state{};
    LongitudinalState efficiency_state{};
    const float normal = run(
        {100.0f, 0.0f, 0.5f, 1000.0f, DriveMode::Normal}, normal_state);
    const float efficiency = run(
        {100.0f, 0.0f, 0.5f, 1000.0f, DriveMode::Efficiency}, efficiency_state);
    TEST_ASSERT_TRUE(efficiency < normal);
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_throttle_drives_positive);
    RUN_TEST(test_brake_override_kills_drive);
    RUN_TEST(test_idle_is_zero);
    RUN_TEST(test_regen_cuts_out_at_80_rpm);
    RUN_TEST(test_regen_smoothstep_at_300_rpm);
    RUN_TEST(test_regen_reaches_60a_mid_speed);
    RUN_TEST(test_regen_uses_inverse_rpm_power_limit);
    RUN_TEST(test_regen_taper_midpoint);
    RUN_TEST(test_regen_cutoff_high_soc_is_immediate);
    RUN_TEST(test_regen_rise_rate_is_10a_per_100ms);
    RUN_TEST(test_regen_fall_rate_is_20a_per_100ms);
    RUN_TEST(test_efficiency_limits_drive);
    return UNITY_END();
}
