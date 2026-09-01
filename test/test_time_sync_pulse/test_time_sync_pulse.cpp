#include <unity.h>
#include "modules/time_sync_pulse.h"

namespace {
TimeSyncPulseParams params() {
    return {true, 20.0f, 0.5f, 0.5f, 3U, 10.0f};
}

TimeSyncPulseInput nominal() {
    return {false, false, false, true, true, 0.01f};
}
}

void test_cannot_run_without_arm(void) {
    TimeSyncPulseState state{};
    auto in = nominal();
    in.run_request = true;
    const auto out = time_sync_pulse_step(in, params(), state);
    TEST_ASSERT_FALSE(out.override_active);
    TEST_ASSERT_EQUAL_INT((int)TimeSyncPulseMode::Disarmed, (int)state.mode);
}

void test_arm_then_run_starts_equal_motor_pulse(void) {
    TimeSyncPulseState state{};
    auto in = nominal();
    in.arm_request = true;
    auto out = time_sync_pulse_step(in, params(), state);
    TEST_ASSERT_TRUE(out.armed);

    in.arm_request = false;
    in.run_request = true;
    out = time_sync_pulse_step(in, params(), state);
    TEST_ASSERT_TRUE(out.override_active);
    TEST_ASSERT_TRUE(out.running);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, out.left_a);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, out.left_a, out.right_a);
}

void test_runtime_fault_aborts_to_zero(void) {
    TimeSyncPulseState state{};
    auto in = nominal();
    in.arm_request = true;
    time_sync_pulse_step(in, params(), state);
    in.arm_request = false;
    in.run_request = true;
    time_sync_pulse_step(in, params(), state);

    in.run_request = false;
    in.runtime_conditions_ok = false;
    const auto out = time_sync_pulse_step(in, params(), state);
    TEST_ASSERT_TRUE(out.aborted_event);
    TEST_ASSERT_FALSE(out.override_active);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.left_a);
    TEST_ASSERT_EQUAL_INT((int)TimeSyncPulseMode::Disarmed, (int)state.mode);
}

void test_three_complete_pulses_finish_disarmed(void) {
    TimeSyncPulseState state{};
    auto in = nominal();
    in.arm_request = true;
    time_sync_pulse_step(in, params(), state);
    in.arm_request = false;
    in.run_request = true;
    time_sync_pulse_step(in, params(), state);
    in.run_request = false;

    bool completed = false;
    unsigned on_samples = 0U;
    for (unsigned i = 0; i < 400U; ++i) {
        const auto out = time_sync_pulse_step(in, params(), state);
        if (out.left_a > 0.0f) ++on_samples;
        if (out.completed_event) completed = true;
    }
    TEST_ASSERT_TRUE(completed);
    TEST_ASSERT_TRUE(on_samples >= 145U && on_samples <= 155U);
    TEST_ASSERT_EQUAL_INT((int)TimeSyncPulseMode::Disarmed, (int)state.mode);
}

void test_arm_times_out(void) {
    TimeSyncPulseState state{};
    auto p = params();
    p.arm_timeout_s = 0.02f;
    auto in = nominal();
    in.arm_request = true;
    time_sync_pulse_step(in, p, state);
    in.arm_request = false;
    auto out = time_sync_pulse_step(in, p, state);
    out = time_sync_pulse_step(in, p, state);
    TEST_ASSERT_TRUE(out.aborted_event);
    TEST_ASSERT_EQUAL_INT((int)TimeSyncPulseMode::Disarmed, (int)state.mode);
}

void test_nonpositive_current_is_rejected(void) {
    TimeSyncPulseState state{};
    auto p = params();
    p.phase_current_per_motor_a = 0.0f;
    auto in = nominal();
    in.arm_request = true;
    time_sync_pulse_step(in, p, state);
    in.arm_request = false;
    in.run_request = true;
    const auto out = time_sync_pulse_step(in, p, state);
    TEST_ASSERT_TRUE(out.aborted_event);
    TEST_ASSERT_FALSE(out.override_active);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_cannot_run_without_arm);
    RUN_TEST(test_arm_then_run_starts_equal_motor_pulse);
    RUN_TEST(test_runtime_fault_aborts_to_zero);
    RUN_TEST(test_three_complete_pulses_finish_disarmed);
    RUN_TEST(test_arm_times_out);
    RUN_TEST(test_nonpositive_current_is_rejected);
    return UNITY_END();
}
