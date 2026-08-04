#include "modules/tv/yaw_control.h"
#include <cmath>

namespace {
float clampf(float value, float lo, float hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}
}

float tv_yaw_compute(float desired_yaw, float measured_yaw, float dt,
                     const TVParams &p, TVYawState &s) {
    if (!std::isfinite(desired_yaw) || !std::isfinite(measured_yaw) ||
        !std::isfinite(dt) || dt <= 0.0f || p.yaw_moment_max <= 0.0f) {
        s.initialized = false;
        return 0.0f;
    }

    float error = desired_yaw - measured_yaw;
    if (std::fabs(error) <= p.yaw_deadband_degps) error = 0.0f;

    const float derivative = s.initialized
        ? -(measured_yaw - s.prev_measured_yaw) / dt
        : 0.0f;
    s.prev_measured_yaw = measured_yaw;
    s.initialized = true;

    const float candidate_integral = s.integral + error * dt;
    const float candidate = p.kp * error + p.ki * candidate_integral +
                            p.kd * derivative;

    // Conditional integration: accept the integral only when it does not
    // drive an already saturated output farther into saturation.
    const bool saturated_high = candidate > p.yaw_moment_max;
    const bool saturated_low = candidate < -p.yaw_moment_max;
    if ((!saturated_high && !saturated_low) ||
        (saturated_high && error < 0.0f) ||
        (saturated_low && error > 0.0f)) {
        s.integral = candidate_integral;
    }

    const float output = p.kp * error + p.ki * s.integral + p.kd * derivative;
    return clampf(output, -p.yaw_moment_max, p.yaw_moment_max);
}
