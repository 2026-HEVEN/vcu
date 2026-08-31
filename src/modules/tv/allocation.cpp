#include "modules/tv/allocation.h"
#include <cmath>

namespace {
float clampf(float value, float lo, float hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}
}

TVAllocOutput tv_alloc_compute(float total_current_a, float yaw_moment_nm,
                               MaxTorque limit, const TVParams &p) {
    if (!std::isfinite(total_current_a) || !std::isfinite(yaw_moment_nm) ||
        !std::isfinite(limit.max_L) || !std::isfinite(limit.max_R) ||
        std::fabs(total_current_a) < 1.0e-6f || p.track_m <= 0.0f ||
        p.tire_radius_m <= 0.0f || p.gear_ratio <= 0.0f ||
        p.motor_kt_nm_per_a <= 0.0f || p.motor_current_max_a <= 0.0f) {
        return {Amp(0.0f), Amp(0.0f)};
    }

    const float max_l = clampf(limit.max_L, 0.0f, p.motor_current_max_a);
    const float max_r = clampf(limit.max_R, 0.0f, p.motor_current_max_a);

    // With I_L=base-diff and I_R=base+diff:
    // Mz=(F_R-F_L)*track/2 = diff*Kt*gear*track/tire_radius.
    float diff = yaw_moment_nm * p.tire_radius_m /
                 (p.track_m * p.motor_kt_nm_per_a * p.gear_ratio);

    float lo_l, hi_l, lo_r, hi_r;
    if (total_current_a > 0.0f) {
        lo_l = lo_r = 0.0f;
        hi_l = max_l; hi_r = max_r;
        diff = clampf(diff, -0.5f * max_l, 0.5f * max_r);
    } else {
        lo_l = -max_l; hi_l = 0.0f;
        lo_r = -max_r; hi_r = 0.0f;
        diff = clampf(diff, -0.5f * max_r, 0.5f * max_l);
    }

    // No-add safety: the motor-limit clamps above bound diff by how much
    // current a single motor can carry, but say nothing about how much the
    // *driver* actually asked for. Left unchecked, a small total_current_a
    // (e.g. coasting) combined with a large yaw_moment_nm demand pulls base
    // toward base_lo/base_hi below, which can be far from 0.5*total_current_a
    // -- i.e. current_l+current_r ends up *manufacturing* current the driver
    // never requested (reproduced: total=1A, mz=100Nm -> sum came out ~93A).
    // Capping |diff| at half the driver's own total demand guarantees
    // current_l+current_r never exceeds total_current_a in magnitude: TV
    // authority degrades gracefully at low throttle instead of inventing
    // propulsion/regen current from nothing.
    const float total_half = 0.5f * std::fabs(total_current_a);
    diff = clampf(diff, -total_half, total_half);

    const float base_lo = (lo_l + diff) > (lo_r - diff)
        ? (lo_l + diff) : (lo_r - diff);
    const float base_hi = (hi_l + diff) < (hi_r - diff)
        ? (hi_l + diff) : (hi_r - diff);
    const float base = clampf(0.5f * total_current_a, base_lo, base_hi);

    const float current_l = clampf(base - diff, lo_l, hi_l);
    const float current_r = clampf(base + diff, lo_r, hi_r);
    return {Amp(current_l), Amp(current_r)};
}
