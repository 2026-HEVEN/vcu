#include "modules/motor_direction.h"
#include "modules/realcar_calibration.h"
#include <cmath>

// Efficiency mode (the Cluster's Paddock switch). The pedal scales the current
// ceiling; the target-speed field always carries the fixed cap. Below the cap
// the speed loop saturates at the ceiling and behaves like torque control, and
// a partial lift never lowers the target into an active speed-loop brake.
MotorDirectionCommand paddock_motor_request(float throttle_pct, Gear gear,
    float max_speed_kph, float current_limit_a, float rolling_radius_m,
    float gear_ratio) {
    if (!std::isfinite(throttle_pct) || !std::isfinite(max_speed_kph) ||
        !std::isfinite(current_limit_a) || !std::isfinite(rolling_radius_m) ||
        !std::isfinite(gear_ratio) || throttle_pct <= 0.0f ||
        max_speed_kph <= 0.0f || current_limit_a <= 0.0f ||
        rolling_radius_m <= 0.0f || gear_ratio <= 0.0f ||
        (gear != Gear::Drive && gear != Gear::Reverse)) return {0, 0, false};
    const float cap_kph = gear == Gear::Reverse
        ? std::fmin(max_speed_kph, realcar_cal::bringup::PADDOCK_REVERSE_MAX_SPEED_KPH)
        : max_speed_kph;
    const float rpm = (cap_kph / 3.6f) * 60.0f * gear_ratio /
        (6.283185307f * rolling_radius_m);
    // Floor rather than round up so the cap is never exceeded by rounding.
    const int target = static_cast<int>(std::fmin(rpm, 32000.0f));
    const float current = current_limit_a * std::fmin(throttle_pct, 100.0f) / 100.0f;
    // Pedal noise near 0 % must not leave the controller a target with a
    // trickle current; below 1 A this is treated as a released pedal.
    if (target < 1 || current < 1.0f) return {0, 0, false};
    const int sign = gear == Gear::Drive ? 1 : -1;
    return {sign * current, sign * target, true};
}

float directional_current(float demand_a, Gear gear, bool direction_armed) {
    if (!direction_armed || !std::isfinite(demand_a)) return 0.0f;
    if (gear == Gear::Drive) return demand_a; // preserve regenerative sign
    if (gear == Gear::Reverse && demand_a > 0.0f) return -demand_a;
    return 0.0f; // reverse braking, Neutral, Park and invalid gear
}

bool regen_forward_rotation_ok(int left_rpm, int right_rpm,
                               bool feedback_fresh, int min_rpm) {
    return feedback_fresh && min_rpm >= 0 &&
        left_rpm > min_rpm && right_rpm < -min_rpm;
}

MotorDirectionCommand motor_direction_command(
    float current_a, Gear gear, bool running, bool regen_allowed,
    bool forward_rotation) {
    if (!running || !std::isfinite(current_a) ||
        (gear != Gear::Drive && gear != Gear::Reverse)) return {0.0f, 0, false};

    if (gear == Gear::Reverse) {
        // Reverse propulsion is negative current, NOT regeneration.
        if (current_a >= 0.0f) return {0.0f, 0, true};
        return {current_a, -4000, true};
    }
    if (current_a < 0.0f) {
        if (!regen_allowed || !forward_rotation)
            return {0.0f, 0, true};
        return {current_a, 0, true}; // verified forward regen wire command
    }
    if (current_a == 0.0f) return {0.0f, 0, true};
    return {current_a, 4000, true};
}
