// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/brake.h"
#include <cmath>

BrakeOutput brake_compute(const BrakeInput &in, const BrakeCalib &calib,
                          BrakeFilterState &state) {
    BrakeOutput o{};
    const bool calibration_valid =
        calib.valid_min_adc < calib.zero_adc &&
        calib.zero_adc < calib.full_scale_adc &&
        calib.full_scale_adc < calib.valid_max_adc &&
        calib.valid_max_adc <= 4095U &&
        std::isfinite(calib.full_scale_bar) && calib.full_scale_bar > 0.0f &&
        std::isfinite(calib.sample_period_s) && calib.sample_period_s > 0.0f &&
        std::isfinite(calib.filter_time_constant_s) &&
        calib.filter_time_constant_s >= 0.0f &&
        std::isfinite(calib.active_on_threshold_pct) &&
        std::isfinite(calib.active_off_threshold_pct) &&
        calib.active_off_threshold_pct >= 0.0f &&
        calib.active_off_threshold_pct < calib.active_on_threshold_pct &&
        calib.active_on_threshold_pct <= 100.0f;
    const bool raw_valid = in.raw_adc >= (int)calib.valid_min_adc &&
        in.raw_adc <= (int)calib.valid_max_adc;
    if (!calibration_valid || !raw_valid) {
        state = BrakeFilterState{};
        return o;
    }

    if (!state.initialized || !std::isfinite(state.filtered_adc)) {
        state.filtered_adc = (float)in.raw_adc;
        state.initialized = true;
    } else {
        const float alpha = calib.filter_time_constant_s <= 0.0f
            ? 1.0f
            : calib.sample_period_s /
                (calib.filter_time_constant_s + calib.sample_period_s);
        state.filtered_adc += alpha * ((float)in.raw_adc - state.filtered_adc);
    }

    float fraction = (state.filtered_adc - (float)calib.zero_adc) /
        (float)(calib.full_scale_adc - calib.zero_adc);
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;

    o.pct = Pct0to100(fraction * 100.0f);
    o.filtered_adc = state.filtered_adc;
    o.pressure_bar = fraction * calib.full_scale_bar;
    if (!state.active && (float)o.pct > calib.active_on_threshold_pct) {
        state.active = true;
    } else if (state.active &&
               (float)o.pct <= calib.active_off_threshold_pct) {
        state.active = false;
    }
    o.active = state.active;
    o.valid = true;
    return o;
}
