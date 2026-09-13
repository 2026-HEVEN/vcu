// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/brake.h"
#include <cmath>

BrakeOutput brake_compute(const BrakeInput &in, const BrakeCalib &calib) {
    BrakeOutput o{};
    const bool calibration_valid =
        calib.valid_min_adc < calib.zero_adc &&
        calib.zero_adc < calib.full_scale_adc &&
        calib.full_scale_adc < calib.valid_max_adc &&
        calib.valid_max_adc <= 4095U &&
        std::isfinite(calib.full_scale_bar) && calib.full_scale_bar > 0.0f &&
        std::isfinite(calib.active_threshold_pct) &&
        calib.active_threshold_pct >= 0.0f &&
        calib.active_threshold_pct <= 100.0f;
    const bool raw_valid = in.raw_adc >= (int)calib.valid_min_adc &&
        in.raw_adc <= (int)calib.valid_max_adc;
    if (!calibration_valid || !raw_valid) return o;

    float fraction = (float)(in.raw_adc - (int)calib.zero_adc) /
        (float)(calib.full_scale_adc - calib.zero_adc);
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;

    o.pct = Pct0to100(fraction * 100.0f);
    o.pressure_bar = fraction * calib.full_scale_bar;
    o.active = (float)o.pct > calib.active_threshold_pct;
    o.valid = true;
    return o;
}
