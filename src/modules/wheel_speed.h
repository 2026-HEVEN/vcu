#pragma once
#include <cstdint>
#include "types.h"
// [FILL-IN] Pure pulse-to-speed conversion. Calibration owned by the team.

struct WssReading { uint32_t pulse_delta; uint32_t dt_ms; };
struct WssCalib {
    float pulses_per_rev;
    float filter_time_constant_s;
    constexpr WssCalib(float ppr = 0.0f, float tau_s = 0.0f)
        : pulses_per_rev(ppr), filter_time_constant_s(tau_s) {}
};
struct WheelSpeedFilterState {
    float rpm = 0.0f;
    bool initialized = false;
};

Rpm wheel_speed_compute(const WssReading &r, const WssCalib &c);
Rpm wheel_speed_compute_filtered(const WssReading &r, const WssCalib &c,
                                 WheelSpeedFilterState &state);
