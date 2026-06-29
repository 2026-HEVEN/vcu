// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/throttle.h"

Percent throttle_compute(const ThrottleInput &in) {
    constexpr float ADC_MAX = 4095.0f;
    constexpr float DEADZONE = 0.05f;            // lower 5% -> 0
    float frac = (float)in.raw_adc / ADC_MAX;
    if (frac < DEADZONE) return Percent(0.0f);
    float pct = (frac - DEADZONE) / (1.0f - DEADZONE) * 100.0f;
    return Percent(pct);                         // type clamps to 0..100
}
