#include "modules/tv/yaw_control.h"
#include <cmath>

namespace {
float clampf(float value, float lo, float hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}
}

float tv_yaw_compute(float desired_yaw, float measured_yaw, float dt,
                     const TVParams &p, TVYawState &s, bool measurement_new) {
    if (!std::isfinite(desired_yaw) || !std::isfinite(measured_yaw) ||
        !std::isfinite(dt) || dt <= 0.0f || p.yaw_moment_max <= 0.0f) {
        s.initialized = false;
        return 0.0f;
    }

    float error = desired_yaw - measured_yaw;
    // Continuous deadband avoids a step at the noise threshold. A negative
    // deadband would flip the sign of small errors, so treat it as zero.
    const float deadband =
        p.yaw_deadband_degps > 0.0f ? p.yaw_deadband_degps : 0.0f;
    if (error > deadband) {
        error -= deadband;
    } else if (error < -deadband) {
        error += deadband;
    } else {
        error = 0.0f;
    }

    // Derivative on measurement. When the control tick re-reads a held IMU
    // sample, the slope is taken over the real time between samples instead
    // of producing D=0 followed by a doubled step. A first-order low-pass then
    // limits the 1/dt noise gain.
    if (!s.initialized) {
        s.prev_measured_yaw = measured_yaw;
        s.derivative = 0.0f;
        s.sample_dt = 0.0f;
        s.initialized = true;
    } else {
        s.sample_dt += dt;
        if (measurement_new) {
            const float raw_derivative =
                -(measured_yaw - s.prev_measured_yaw) / s.sample_dt;
            const float tau = p.derivative_filter_tau_s > 0.0f
                ? p.derivative_filter_tau_s : 0.0f;
            const float alpha = s.sample_dt / (tau + s.sample_dt);
            s.derivative += alpha * (raw_derivative - s.derivative);
            s.prev_measured_yaw = measured_yaw;
            s.sample_dt = 0.0f;
        }
    }
    const float derivative = s.derivative;

    // The integral state is also capped so that ki * integral alone can never
    // exceed the Mz limit; otherwise integral_max means nothing for large ki
    // and too little for small ki.
    float integral_limit = p.integral_max > 0.0f ? p.integral_max : 0.0f;
    if (p.ki > 1.0e-6f) {
        const float authority_limit = p.yaw_moment_max / p.ki;
        if (authority_limit < integral_limit) integral_limit = authority_limit;
    }
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
    return clampf(output, -p.yaw_moment_max, p.yaw_moment_max);
}
