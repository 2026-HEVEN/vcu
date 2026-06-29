#include "modules/wheel_speed.h"

Rpm wheel_speed_compute(const WssReading &r, const WssCalib &c) {
    if (r.dt_ms == 0 || c.pulses_per_rev <= 0.0f) return Rpm(0.0f);
    float revs = (float)r.pulse_delta / c.pulses_per_rev;
    float rpm  = revs / ((float)r.dt_ms / 1000.0f) * 60.0f;
    return Rpm(rpm);
}
