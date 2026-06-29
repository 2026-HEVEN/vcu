// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include "types.h"
// [LOCKED] Shared state bus. ONLY core/app_wiring.cpp may include this.
// Module files (src/modules/*) must never include state.h.

struct VehicleState {
    // inputs
    Percent   throttle_pct;       // 0..100 (clamped both ways)
    Pct0to100 brake_pct;
    bool      brake_active = false;
    Unit      steering_angle;     // -1..+1
    float     yaw_rate = 0.0f;    // deg/s
    float     accel_x  = 0.0f;
    float     accel_y  = 0.0f;
    Rpm       wheel_speed;
    float     pack_soc = 0.0f;    // 0..1
    // controller feedback (from CAN)
    Rpm       motor_speed_L;
    Rpm       motor_speed_R;
    // control outputs
    float     total_torque = 0.0f;   // signed A demand
    Percent   torque_L;
    Percent   torque_R;
};

extern VehicleState state;   // defined in core/app_wiring.cpp
