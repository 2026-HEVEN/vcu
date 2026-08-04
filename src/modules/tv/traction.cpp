#include "modules/tv/traction.h"
#include <cmath>

namespace {
constexpr float G_MPS2 = 9.80665f;

float clampf(float value, float lo, float hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}
}

MaxTorque tv_traction_compute(WheelLoads fz, float ay_g, const TVParams &p) {
    if (!std::isfinite(fz.fz_L) || !std::isfinite(fz.fz_R) ||
        !std::isfinite(ay_g) || p.mu <= 0.0f || p.tire_radius_m <= 0.0f ||
        p.gear_ratio <= 0.0f || p.motor_kt_nm_per_a <= 0.0f ||
        p.motor_current_max_a <= 0.0f) return {0.0f, 0.0f};

    const float left_fz = fz.fz_L > 0.0f ? fz.fz_L : 0.0f;
    const float right_fz = fz.fz_R > 0.0f ? fz.fz_R : 0.0f;
    const float rear_fz = left_fz + right_fz;
    if (rear_fz <= 0.0f) return {0.0f, 0.0f};

    const float rear_fy = p.mass_kg * std::fabs(ay_g) * G_MPS2 * p.weight_dist_r;
    const float fy_left = rear_fy * left_fz / rear_fz;
    const float fy_right = rear_fy * right_fz / rear_fz;

    const auto current_limit = [&](float wheel_fz, float wheel_fy) {
        const float friction = p.mu * wheel_fz;
        const float remaining_sq = friction * friction - wheel_fy * wheel_fy;
        if (remaining_sq <= 0.0f) return 0.0f;
        const float wheel_torque = std::sqrt(remaining_sq) * p.tire_radius_m;
        const float motor_current = wheel_torque /
            (p.gear_ratio * p.motor_kt_nm_per_a);
        return clampf(motor_current, 0.0f, p.motor_current_max_a);
    };

    return {current_limit(left_fz, fy_left),
            current_limit(right_fz, fy_right)};
}
