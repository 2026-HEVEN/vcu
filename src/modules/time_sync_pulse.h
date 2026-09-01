#pragma once

struct TimeSyncPulseParams {
    bool enabled;
    float phase_current_per_motor_a;
    float pulse_on_s;
    float pulse_off_s;
    unsigned pulse_count;
    float arm_timeout_s;
};

struct TimeSyncPulseInput {
    bool arm_request;
    bool run_request;
    bool cancel_request;
    bool start_conditions_ok;
    bool runtime_conditions_ok;
    float dt_s;
};

enum class TimeSyncPulseMode {
    Disarmed,
    Armed,
    PulseOn,
    PulseOff,
};

struct TimeSyncPulseState {
    TimeSyncPulseMode mode = TimeSyncPulseMode::Disarmed;
    float elapsed_s = 0.0f;
    unsigned completed_pulses = 0;
};

struct TimeSyncPulseOutput {
    float left_a = 0.0f;
    float right_a = 0.0f;
    bool override_active = false;
    bool armed = false;
    bool running = false;
    bool completed_event = false;
    bool aborted_event = false;
};

TimeSyncPulseOutput time_sync_pulse_step(
    const TimeSyncPulseInput &in,
    const TimeSyncPulseParams &params,
    TimeSyncPulseState &state);
