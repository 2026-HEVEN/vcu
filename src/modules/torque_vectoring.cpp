#include "modules/torque_vectoring.h"

TVOutput tv_compute(const TVInput &in) {
    // SAFE STUB: even 50/50 split. TODO(team-torquevectoring): bias L/R using
    // yaw_rate vs steering_angle (and wheel_speed for slip) to add yaw moment.
    float half = in.total_torque * 0.5f;
    return TVOutput{ Percent(half), Percent(half) };
}
