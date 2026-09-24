#include "modules/can_link_policy.h"
#include "modules/fixed_config.h"

namespace {
namespace rt = fixed_config::runtime;

bool within(bool seen, uint32_t age_ms, uint32_t limit_ms) {
    return seen && age_ms <= limit_ms;
}

int32_t wrap(int32_t v, int32_t period) {
    const int32_t r = v % period;
    return r < 0 ? r + period : r;
}

bool all_in(const CmdPhaseInput &in, int32_t shift, int32_t lo, int32_t hi) {
    const int32_t period = (int32_t)rt::MOTOR_COMMAND_PERIOD_MS;
    // Sending later by `shift` brings the next FB1 closer by the same amount.
    if (in.known_L) {
        const int32_t p = wrap(in.phase_L_ms - shift, period);
        if (p < lo || p > hi) return false;
    }
    if (in.known_R) {
        const int32_t p = wrap(in.phase_R_ms - shift, period);
        if (p < lo || p > hi) return false;
    }
    return true;
}

int32_t smallest_shift(const CmdPhaseInput &in, int32_t lo, int32_t hi, bool &found) {
    const int32_t half = (int32_t)rt::MOTOR_COMMAND_PERIOD_MS / 2;
    for (int32_t d = 0; d <= half; ++d) {
        if (all_in(in, d, lo, hi))  { found = true; return d; }
        if (all_in(in, -d, lo, hi)) { found = true; return -d; }
    }
    found = false;
    return 0;
}
}  // namespace

bool controller_feedback_fresh(const FeedbackAges &a) {
    const uint32_t stale = (uint32_t)rt::CONTROLLER_FEEDBACK_STALE_MS;
    const bool recent = within(a.fb1_seen, a.fb1_age_ms, stale) ||
                        within(a.fb2_seen, a.fb2_age_ms, stale);
    return recent &&
           within(a.fb1_seen, a.fb1_age_ms, rt::CONTROLLER_FB1_MAX_AGE_MS) &&
           within(a.fb2_seen, a.fb2_age_ms, rt::CONTROLLER_FB2_MAX_AGE_MS);
}

bool controller_feedback_lost(const FeedbackAges &a) {
    const uint32_t silent = rt::CONTROLLER_REHANDSHAKE_TIMEOUT_MS;
    const bool both_silent = !within(a.fb1_seen, a.fb1_age_ms, silent) &&
                             !within(a.fb2_seen, a.fb2_age_ms, silent);
    return both_silent ||
           !within(a.fb1_seen, a.fb1_age_ms, rt::CONTROLLER_FB1_MAX_AGE_MS) ||
           !within(a.fb2_seen, a.fb2_age_ms, rt::CONTROLLER_FB2_MAX_AGE_MS);
}

bool controller_feedback_both_recent(const FeedbackAges &a) {
    const uint32_t stale = (uint32_t)rt::CONTROLLER_FEEDBACK_STALE_MS;
    return within(a.fb1_seen, a.fb1_age_ms, stale) &&
           within(a.fb2_seen, a.fb2_age_ms, stale);
}

int32_t cmd_phase_of(uint32_t fb1_rx_ms, uint32_t send_ms, uint32_t period_ms) {
    // Signed difference survives millis() rollover. Folding the unsigned
    // difference directly would be wrong: 2^32 is not a multiple of 50.
    return wrap((int32_t)(fb1_rx_ms - send_ms), (int32_t)period_ms);
}

int32_t cmd_phase_shift_ms(const CmdPhaseInput &in) {
    if (!in.known_L && !in.known_R) return 0;
    if (all_in(in, 0, rt::CMD_PHASE_SAFE_MIN_MS, rt::CMD_PHASE_SAFE_MAX_MS)) return 0;
    bool found = false;
    int32_t d = smallest_shift(in, rt::CMD_PHASE_TARGET_MIN_MS,
                               rt::CMD_PHASE_TARGET_MAX_MS, found);
    if (found) return d;
    // Phases too far apart for the target band: the safe band always fits.
    d = smallest_shift(in, rt::CMD_PHASE_SAFE_MIN_MS, rt::CMD_PHASE_SAFE_MAX_MS, found);
    return found ? d : 0;
}

int32_t cmd_phase_step(CmdPhaseState &st, const CmdPhaseInput &in) {
    const int32_t d = cmd_phase_shift_ms(in);
    if (d == 0) {
        st.unsafe_ticks = 0U;
        return 0;
    }
    if (++st.unsafe_ticks < rt::CMD_PHASE_CONFIRM_TICKS) return 0;
    st.unsafe_ticks = 0U;
    return d;
}
