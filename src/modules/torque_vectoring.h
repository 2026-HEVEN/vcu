#pragma once
#include "types.h"
// [FILL-IN] Lateral torque split. Takes a signed total torque, returns L/R commands.

struct TVInput  { float total_torque; float yaw_rate; float steering_angle; float wheel_speed; };
struct TVOutput { Percent torque_L; Percent torque_R; };

TVOutput tv_compute(const TVInput &in);
