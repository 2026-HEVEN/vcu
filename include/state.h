// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include "types.h"
#include "modules/vehicle_speed.h"   // WheelIdx / WHEEL_COUNT
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
    bool      imu_valid = false;    // complete Acceleration + RateOfTurn, fresh <=100 ms
    Rpm       wheel_speed[WHEEL_COUNT];   // FL, FR, RL, RR (개별 휠속)
    float     vehicle_speed_mps  = 0.0f;  // 전륜 기준 추정 차속 (vehicle_speed 모듈)
    bool      vehicle_speed_valid = false;// false = 전륜 신호 불신 → TV 비활성
    float     pack_soc = 0.0f;    // 0..1
    // controller feedback (from CAN)
    Rpm       motor_speed_L;
    Rpm       motor_speed_R;
    // control outputs
    float     total_torque = 0.0f;   // signed A demand
    Percent   torque_L;
    Percent   torque_R;
    // TV intermediate signals (관측/튜닝용; app_wiring이 TVOutput에서 복사)
    float     desired_yaw_rate = 0.0f;   // reference stage
    float     yaw_moment       = 0.0f;   // yaw_control stage (Mz)
    float     fz_L = 0.0f, fz_R = 0.0f;  // load stage (바퀴별 수직하중)
    float     max_torque_L = 0.0f, max_torque_R = 0.0f;  // traction stage
};

extern VehicleState state;   // defined in core/app_wiring.cpp
