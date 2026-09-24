#pragma once
#include "modules/gear.h"

// Longitudinal demand: positive = propel, negative = brake (before gear sign).
float directional_current(float demand_a, Gear gear, bool direction_armed);

// Installed forward polarity: left RPM positive, right RPM negative. Both
// fresh feedback streams must exceed the configured motor-RPM threshold.
bool regen_forward_rotation_ok(int left_rpm, int right_rpm,
                               bool feedback_fresh, int min_rpm);

struct MotorDirectionCommand {
    float current_a;
    int target_rpm;
    bool running;
};

// Input current is already gear-signed. Both installed controllers share this
// convention. Regen permission is distinct from a negative reverse-drive current.
MotorDirectionCommand motor_direction_command(
    float current_a, Gear gear, bool running, bool regen_allowed,
    bool forward_rotation);
