#include "modules/motor_direction.h"
#include <cmath>

float directional_current(float demand_a, Gear gear, bool direction_armed) {
    if (!direction_armed || !std::isfinite(demand_a)) return 0.0f;
    if (gear == Gear::Drive) return demand_a; // preserve regenerative sign
    if (gear == Gear::Reverse && demand_a > 0.0f) return -demand_a;
    return 0.0f; // reverse braking, Neutral, Park and invalid gear
}

MotorDirectionCommand motor_direction_command(
    float current_a, Gear gear, bool running, bool brake_active,
    bool regen_allowed, bool forward_rotation) {
    if (!running || !std::isfinite(current_a) ||
        (gear != Gear::Drive && gear != Gear::Reverse)) return {0.0f, 0, false};

    if (gear == Gear::Reverse) {
        // Reverse propulsion is negative current, NOT regeneration.
        if (current_a >= 0.0f) return {0.0f, 0, true};
        return {current_a, -4000, true};
    }
    if (current_a < 0.0f) {
        if (!brake_active || !regen_allowed || !forward_rotation)
            return {0.0f, 0, true};
        return {current_a, 0, true}; // verified forward regen wire command
    }
    if (current_a == 0.0f) return {0.0f, 0, true};
    return {current_a, 4000, true};
}
