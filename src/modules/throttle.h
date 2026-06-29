#pragma once
#include "types.h"
// [FILL-IN] Pure throttle conversion. No hardware, no state.h, no Arduino.h.

struct ThrottleInput { int raw_adc; };   // 0..4095

Percent throttle_compute(const ThrottleInput &in);
