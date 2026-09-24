#include <unity.h>
#include <cstdlib>
#include "modules/can_link_policy.h"
#include "modules/fixed_config.h"

namespace rt = fixed_config::runtime;

static FeedbackAges ages(uint32_t fb1, uint32_t fb2) {
    FeedbackAges a;
    a.fb1_seen = true; a.fb1_age_ms = fb1;
    a.fb2_seen = true; a.fb2_age_ms = fb2;
    return a;
}

// ---- freshness ----
void test_both_parts_recent_is_fresh(void) {
    TEST_ASSERT_TRUE(controller_feedback_fresh(ages(10, 30)));
}
void test_fb1_pause_with_fb2_alive_stays_fresh(void) {
    // 2026-09-24: FB1 stopped 995 ms while FB2 kept arriving.
    TEST_ASSERT_TRUE(controller_feedback_fresh(ages(995, 20)));
    TEST_ASSERT_FALSE(controller_feedback_lost(ages(995, 20)));
}
void test_fb2_pause_with_fb1_alive_stays_fresh(void) {
    // 2026-09-24: FB2 gap 4.5 s, 2026-09-21: up to 6.1 s under load.
    TEST_ASSERT_TRUE(controller_feedback_fresh(ages(20, 6100)));
    TEST_ASSERT_FALSE(controller_feedback_lost(ages(20, 6100)));
}
void test_both_parts_stale_is_not_fresh(void) {
    TEST_ASSERT_FALSE(controller_feedback_fresh(ages(260, 260)));
}
void test_fb1_hard_limit(void) {
    TEST_ASSERT_FALSE(controller_feedback_fresh(ages(rt::CONTROLLER_FB1_MAX_AGE_MS + 1, 10)));
    TEST_ASSERT_TRUE(controller_feedback_lost(ages(rt::CONTROLLER_FB1_MAX_AGE_MS + 1, 10)));
}
void test_fb2_hard_limit(void) {
    TEST_ASSERT_FALSE(controller_feedback_fresh(ages(10, rt::CONTROLLER_FB2_MAX_AGE_MS + 1)));
    TEST_ASSERT_TRUE(controller_feedback_lost(ages(10, rt::CONTROLLER_FB2_MAX_AGE_MS + 1)));
}
void test_both_silent_drops_link(void) {
    const uint32_t t = rt::CONTROLLER_REHANDSHAKE_TIMEOUT_MS + 1;
    TEST_ASSERT_TRUE(controller_feedback_lost(ages(t, t)));
    TEST_ASSERT_FALSE(controller_feedback_lost(ages(t - 2, t)));
}
void test_unseen_part_is_neither_fresh_nor_kept(void) {
    FeedbackAges a = ages(10, 10);
    a.fb2_seen = false;
    TEST_ASSERT_FALSE(controller_feedback_fresh(a));
    TEST_ASSERT_TRUE(controller_feedback_lost(a));
    FeedbackAges none;
    TEST_ASSERT_FALSE(controller_feedback_fresh(none));
    TEST_ASSERT_TRUE(controller_feedback_lost(none));
}

void test_both_recent_is_strict(void) {
    // Relaxed gating keeps torque, but FB2-based decisions need both parts.
    TEST_ASSERT_TRUE(controller_feedback_both_recent(ages(10, 30)));
    TEST_ASSERT_FALSE(controller_feedback_both_recent(ages(10, 400)));
    TEST_ASSERT_FALSE(controller_feedback_both_recent(ages(400, 10)));
    TEST_ASSERT_TRUE(controller_feedback_fresh(ages(10, 400)));
}

// ---- phase measurement ----
void test_phase_of_basic_and_negative(void) {
    TEST_ASSERT_EQUAL_INT32(20, cmd_phase_of(1020, 1000, 50));
    TEST_ASSERT_EQUAL_INT32(40, cmd_phase_of(990, 1000, 50));   // FB1 before send
    TEST_ASSERT_EQUAL_INT32(0, cmd_phase_of(1050, 1000, 50));
}
void test_phase_of_survives_millis_rollover(void) {
    TEST_ASSERT_EQUAL_INT32(15, cmd_phase_of(5u, 0xFFFFFFF6u, 50));  // +15 across wrap
    TEST_ASSERT_EQUAL_INT32(35, cmd_phase_of(0xFFFFFFF6u, 5u, 50));  // -15 across wrap
}

// ---- phase guard ----
static bool in_band(int32_t p, int lo, int hi) { return p >= lo && p <= hi; }
static int32_t after(int32_t p, int32_t shift) {
    int32_t r = (p - shift) % 50; return r < 0 ? r + 50 : r;
}

void test_two_sides_are_centred(void) {
    // 2026-09-24 run: L drifted to 11, R 16. Both should straddle CENTER.
    CmdPhaseInput in; in.known_L = true; in.phase_L_ms = 11; in.known_R = true; in.phase_R_ms = 16;
    const int32_t d = cmd_phase_shift_ms(in);
    const int32_t l = after(11, d), r = after(16, d);
    TEST_ASSERT_TRUE(l >= 26 && l <= 30 && r >= 30 && r <= 34);
}
void test_no_shift_when_safe_or_unknown(void) {
    CmdPhaseInput in; in.known_L = true; in.phase_L_ms = 20; in.known_R = true; in.phase_R_ms = 30;
    TEST_ASSERT_EQUAL_INT32(0, cmd_phase_shift_ms(in));
    CmdPhaseInput none;
    TEST_ASSERT_EQUAL_INT32(0, cmd_phase_shift_ms(none));
}
void test_single_side_in_window_moves_to_target(void) {
    CmdPhaseInput in; in.known_L = true; in.phase_L_ms = 2;
    const int32_t d = cmd_phase_shift_ms(in);
    TEST_ASSERT_EQUAL_INT32(rt::CMD_PHASE_CENTER_MS, after(2, d));
    TEST_ASSERT_EQUAL_INT32(22, d);   // 2 -> 30 by sending 22 ms later
}
void test_every_phase_pair_has_a_safe_shift(void) {
    for (int l = 0; l < 50; ++l) {
        for (int r = 0; r < 50; ++r) {
            CmdPhaseInput in;
            in.known_L = true; in.phase_L_ms = l;
            in.known_R = true; in.phase_R_ms = r;
            const int32_t d = cmd_phase_shift_ms(in);
            TEST_ASSERT_TRUE(std::abs(d) <= 25);
            TEST_ASSERT_TRUE(in_band(after(l, d), rt::CMD_PHASE_SAFE_MIN_MS, rt::CMD_PHASE_SAFE_MAX_MS));
            TEST_ASSERT_TRUE(in_band(after(r, d), rt::CMD_PHASE_SAFE_MIN_MS, rt::CMD_PHASE_SAFE_MAX_MS));
        }
    }
}
void test_measured_window_is_never_left_alone(void) {
    // Real windows ~1-3.5 ms and ~8 ms plus up to 5 ms RX-poll delay -> measured 0-13 ms.
    for (int p = 0; p <= 13; ++p) {
        CmdPhaseInput in; in.known_R = true; in.phase_R_ms = p;
        TEST_ASSERT_TRUE(cmd_phase_shift_ms(in) != 0);
    }
}
void test_step_needs_consecutive_confirmation(void) {
    CmdPhaseState st;
    CmdPhaseInput bad; bad.known_L = true; bad.phase_L_ms = 3;
    CmdPhaseInput good; good.known_L = true; good.phase_L_ms = 25;
    for (unsigned i = 1; i < rt::CMD_PHASE_CONFIRM_TICKS; ++i)
        TEST_ASSERT_EQUAL_INT32(0, cmd_phase_step(st, bad));
    TEST_ASSERT_EQUAL_INT32(0, cmd_phase_step(st, good));   // one good tick resets
    for (unsigned i = 1; i < rt::CMD_PHASE_CONFIRM_TICKS; ++i)
        TEST_ASSERT_EQUAL_INT32(0, cmd_phase_step(st, bad));
    TEST_ASSERT_TRUE(cmd_phase_step(st, bad) != 0);
    TEST_ASSERT_EQUAL_UINT8(0, st.unsafe_ticks);
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_both_parts_recent_is_fresh);
    RUN_TEST(test_fb1_pause_with_fb2_alive_stays_fresh);
    RUN_TEST(test_fb2_pause_with_fb1_alive_stays_fresh);
    RUN_TEST(test_both_parts_stale_is_not_fresh);
    RUN_TEST(test_fb1_hard_limit);
    RUN_TEST(test_fb2_hard_limit);
    RUN_TEST(test_both_silent_drops_link);
    RUN_TEST(test_unseen_part_is_neither_fresh_nor_kept);
    RUN_TEST(test_both_recent_is_strict);
    RUN_TEST(test_phase_of_basic_and_negative);
    RUN_TEST(test_phase_of_survives_millis_rollover);
    RUN_TEST(test_no_shift_when_safe_or_unknown);
    RUN_TEST(test_two_sides_are_centred);
    RUN_TEST(test_single_side_in_window_moves_to_target);
    RUN_TEST(test_every_phase_pair_has_a_safe_shift);
    RUN_TEST(test_measured_window_is_never_left_alone);
    RUN_TEST(test_step_needs_consecutive_confirmation);
    return UNITY_END();
}
