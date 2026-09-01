#include "modules/time_sync_pulse.h"

namespace {
void disarm(TimeSyncPulseState &state) {
    state.mode = TimeSyncPulseMode::Disarmed;
    state.elapsed_s = 0.0f;
    state.completed_pulses = 0;
}

float positive_or_zero(float value) {
    return value > 0.0f ? value : 0.0f;
}
}

TimeSyncPulseOutput time_sync_pulse_step(
    const TimeSyncPulseInput &in,
    const TimeSyncPulseParams &params,
    TimeSyncPulseState &state) {
    TimeSyncPulseOutput out{};
    const bool was_armed_or_running =
        state.mode != TimeSyncPulseMode::Disarmed;
    bool just_armed = false;

    if (!params.enabled || in.cancel_request) {
        out.aborted_event = was_armed_or_running && in.cancel_request;
        disarm(state);
        return out;
    }

    if (in.arm_request) {
        disarm(state);
        if (in.start_conditions_ok) {
            state.mode = TimeSyncPulseMode::Armed;
            just_armed = true;
        }
    }

    if (state.mode == TimeSyncPulseMode::Armed) {
        if (!in.runtime_conditions_ok) {
            out.aborted_event = true;
            disarm(state);
            return out;
        }
        if (in.run_request && !just_armed) {
            if (!in.start_conditions_ok || params.pulse_count == 0U ||
                params.pulse_on_s <= 0.0f || params.pulse_off_s < 0.0f) {
                out.aborted_event = true;
                disarm(state);
                return out;
            }
            if (params.phase_current_per_motor_a <= 0.0f) {
                out.aborted_event = true;
                disarm(state);
                return out;
            }
            state.mode = TimeSyncPulseMode::PulseOn;
            state.elapsed_s = 0.0f;
            state.completed_pulses = 0U;
        } else if (!just_armed) {
            state.elapsed_s += positive_or_zero(in.dt_s);
            if (state.elapsed_s >= params.arm_timeout_s) {
                out.aborted_event = true;
                disarm(state);
                return out;
            }
        }
    }

    if (state.mode == TimeSyncPulseMode::PulseOn ||
        state.mode == TimeSyncPulseMode::PulseOff) {
        if (!in.runtime_conditions_ok) {
            out.aborted_event = true;
            disarm(state);
            return out;
        }

        const bool pulse_on = state.mode == TimeSyncPulseMode::PulseOn;
        out.override_active = true;
        out.running = true;
        if (pulse_on) {
            out.left_a = params.phase_current_per_motor_a;
            out.right_a = params.phase_current_per_motor_a;
        }

        state.elapsed_s += positive_or_zero(in.dt_s);
        const float phase_duration = pulse_on
            ? params.pulse_on_s : params.pulse_off_s;
        if (state.elapsed_s >= phase_duration) {
            state.elapsed_s = 0.0f;
            if (pulse_on) {
                ++state.completed_pulses;
                state.mode = TimeSyncPulseMode::PulseOff;
            } else if (state.completed_pulses >= params.pulse_count) {
                out.completed_event = true;
                disarm(state);
            } else {
                state.mode = TimeSyncPulseMode::PulseOn;
            }
        }
    }

    out.armed = state.mode == TimeSyncPulseMode::Armed;
    return out;
}
