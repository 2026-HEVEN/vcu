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

// Worst distance from CENTER over the known phases after `shift`.
int32_t off_center(const CmdPhaseInput &in, int32_t shift) {
    const int32_t period = (int32_t)rt::MOTOR_COMMAND_PERIOD_MS;
    int32_t worst = 0;
    if (in.known_L) {
        const int32_t e = wrap(in.phase_L_ms - shift, period) - rt::CMD_PHASE_CENTER_MS;
        worst = e < 0 ? -e : e;
    }
    if (in.known_R) {
        const int32_t e = wrap(in.phase_R_ms - shift, period) - rt::CMD_PHASE_CENTER_MS;
        const int32_t a = e < 0 ? -e : e;
        if (a > worst) worst = a;
    }
    return worst;
}

// Among shifts that put every known phase in the safe band, the one that
// keeps the phases closest to CENTER; ties go to the smaller move.
int32_t centered_shift(const CmdPhaseInput &in, bool &found) {
    const int32_t half = (int32_t)rt::MOTOR_COMMAND_PERIOD_MS / 2;
    found = false;
    int32_t best = 0, best_cost = 0;
    for (int32_t mag = 0; mag <= half; ++mag) {
        for (int32_t sign = 1; sign >= -1; sign -= 2) {
            const int32_t d = sign * mag;
            if (!all_in(in, d, rt::CMD_PHASE_SAFE_MIN_MS, rt::CMD_PHASE_SAFE_MAX_MS)) continue;
            const int32_t cost = off_center(in, d);
            if (!found || cost < best_cost) { found = true; best = d; best_cost = cost; }
            if (mag == 0) break;
        }
    }
    return best;
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
    const int32_t d = centered_shift(in, found);
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
