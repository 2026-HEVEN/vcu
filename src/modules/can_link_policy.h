#pragma once
#include <cstdint>

// EZkontrol link policy: when feedback counts as alive, and when to move the
// motor command pair so it never lands in the controllers' feedback-drop
// window. Pure functions; can_bus.cpp owns the clocks and the state.
// Background: docs/CAN_PHASE_GUARD.md

// ---- Feedback freshness ---------------------------------------------------
// Ages are measured by the caller at one instant. `seen` is false until the
// first frame after boot or after the link was invalidated.
struct FeedbackAges {
    bool     fb1_seen = false;
    uint32_t fb1_age_ms = 0U;
    bool     fb2_seen = false;
    uint32_t fb2_age_ms = 0U;
};

// Torque may flow. One recent part is enough, because each part is dropped
// independently by the controller; each part still has a hard age limit.
bool controller_feedback_fresh(const FeedbackAges &a);

// Drop the link and wait for a new 0x55 handshake.
bool controller_feedback_lost(const FeedbackAges &a);

// Both parts recent. Required wherever a decision reads FB2 fault/temperature
// values (fault re-arm, component test), so an old "no fault" is never trusted.
bool controller_feedback_both_recent(const FeedbackAges &a);

// ---- Command phase guard --------------------------------------------------
// phase = time from the command pair's send until that controller's next FB1,
// in [0, period). Measured phases include up to one RX-poll period of delay.
int32_t cmd_phase_of(uint32_t fb1_rx_ms, uint32_t send_ms, uint32_t period_ms);

struct CmdPhaseInput {
    bool    known_L = false;
    int32_t phase_L_ms = 0;
    bool    known_R = false;
    int32_t phase_R_ms = 0;
};

// Shift (ms) to apply to the next send once a known phase has left the safe
// band: every known phase lands in the safe band, as close to CENTER as
// possible. Positive = send later. 0 when no shift is needed or nothing is known.
int32_t cmd_phase_shift_ms(const CmdPhaseInput &in);

// Debounced wrapper: a phase must be outside the safe band for several
// consecutive command periods before the pair is moved.
struct CmdPhaseState {
    uint8_t unsafe_ticks = 0U;
};
int32_t cmd_phase_step(CmdPhaseState &st, const CmdPhaseInput &in);
