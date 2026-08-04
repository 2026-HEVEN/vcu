#include "modules/tv/load.h"
#include <cmath>

namespace {
constexpr float G_MPS2 = 9.80665f;
float nonnegative(float value) { return value > 0.0f ? value : 0.0f; }
}

WheelLoads tv_load_compute(float ax_g, float ay_g, const TVParams &p) {
    if (!std::isfinite(ax_g) || !std::isfinite(ay_g) || p.mass_kg <= 0.0f ||
        p.wheelbase_m <= 0.0f || p.track_m <= 0.0f) return {0.0f, 0.0f};

    const float rear_static = p.mass_kg * G_MPS2 * p.weight_dist_r;
    const float rear_long_transfer =
        p.mass_kg * (ax_g * G_MPS2) * p.cg_height_m / p.wheelbase_m;
    const float rear_total = nonnegative(rear_static + rear_long_transfer);

    // dFz_lr is the right-minus-left rear load difference. Using +/- half
    // avoids the factor-of-two ambiguity in the original draft.
    const float dFz_lr = p.lltd_r * p.mass_kg * (ay_g * G_MPS2) *
                         p.cg_height_m / p.track_m;
    return {
        nonnegative(0.5f * (rear_total - dFz_lr)),
        nonnegative(0.5f * (rear_total + dFz_lr)),
    };
}
