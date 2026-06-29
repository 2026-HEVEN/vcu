#pragma once
#include <cstdint>
#include "types.h"
// [FILL-IN] Pure pulse-to-speed conversion. Calibration owned by the team.

struct WssReading { uint32_t pulse_delta; uint32_t dt_ms; };
struct WssCalib   { float pulses_per_rev; };

Rpm wheel_speed_compute(const WssReading &r, const WssCalib &c);
