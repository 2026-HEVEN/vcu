#include "modules/brake.h"

BrakeOutput brake_compute(const BrakeInput &in) {
    float pct = (float)in.raw_adc / 4095.0f * 100.0f;
    BrakeOutput o;
    o.pct = Pct0to100(pct);
    o.active = pct > 8.0f;
    return o;
}
