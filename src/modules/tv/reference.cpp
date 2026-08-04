#include "modules/tv/reference.h"
#include <cmath>

namespace {
constexpr float RAD_TO_DEG = 57.2957795f;
constexpr float G_MPS2 = 9.80665f;

float clampf(float value, float lo, float hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}
}

float tv_reference_compute(Unit steering, float speed_mps, const TVParams &p) {
    if (!std::isfinite(speed_mps) || !std::isfinite((float)steering) ||
        p.wheelbase_m <= 0.0f || p.desired_yaw_max <= 0.0f) return 0.0f;

    const float speed = std::fabs(speed_mps);
    if (speed < p.tv_min_speed_mps) return 0.0f;

    const float delta = (float)steering * p.max_steer_rad;
    const float denominator = p.wheelbase_m + p.understeer_grad * speed * speed;
    if (denominator <= 1.0e-6f) return 0.0f;

    float desired_radps = speed * delta / denominator;
    // The reference must not request more lateral acceleration than mu*g.
    if (p.mu > 0.0f) {
        const float friction_yaw_limit = p.mu * G_MPS2 / speed;
        desired_radps = clampf(desired_radps, -friction_yaw_limit, friction_yaw_limit);
    }
    return clampf(desired_radps * RAD_TO_DEG,
                  -p.desired_yaw_max, p.desired_yaw_max);
}
