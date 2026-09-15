#include "modules/tv/yaw_control.h"
#include <cmath>

namespace {
float clampf(float value, float lo, float hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}
}

NewtonMetre tv_yaw_compute(DegPerSec desired_yaw_q, DegPerSec measured_yaw_q,
                           Seconds dt_q, const TVParams &p, TVYawState &s) {
    // 경계에서 한 번 벗기고 이후는 생 float로 계산한다.
    const float desired_yaw  = (float)desired_yaw_q;
    const float measured_yaw = (float)measured_yaw_q;
    const float dt           = (float)dt_q;
    if (!std::isfinite(desired_yaw) || !std::isfinite(measured_yaw) ||
        !std::isfinite(dt) || dt <= 0.0f || p.yaw_moment_max <= 0.0f) {
        s.initialized = false;
        return NewtonMetre{};
    }

    float error = desired_yaw - measured_yaw;
    // Continuous deadband avoids a step at the noise threshold.
    if (error > p.yaw_deadband_degps) {
        error -= p.yaw_deadband_degps;
    } else if (error < -p.yaw_deadband_degps) {
        error += p.yaw_deadband_degps;
    } else {
        error = 0.0f;
    }

    const float derivative = s.initialized
        ? -(measured_yaw - s.prev_measured_yaw) / dt
        : 0.0f;
    s.prev_measured_yaw = measured_yaw;
    s.initialized = true;

    const float integral_limit = p.integral_max > 0.0f ? p.integral_max : 0.0f;
    const float candidate_integral = clampf(
        s.integral + error * dt, -integral_limit, integral_limit);
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
    return NewtonMetre{clampf(output, -p.yaw_moment_max, p.yaw_moment_max)};
}
