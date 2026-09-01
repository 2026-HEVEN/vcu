// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include "types.h"
#include "can_protocol.h"
#include "modules/gear.h"
#include "modules/vehicle_speed.h"   // WheelIdx / WHEEL_COUNT
// [LOCKED] Shared state bus. ONLY core/app_wiring.cpp may include this.
// Module files (src/modules/*) must never include state.h.

struct VehicleState {
    // inputs
    int       throttle_raw_adc = 0; // diagnostics/calibration; 0..4095
    Percent   throttle_pct;       // 0..100 (clamped both ways)
    Pct0to100 brake_pct;
    bool      brake_active = false;
    Unit      steering_angle;     // -1..+1
    float     yaw_rate = 0.0f;    // deg/s
    float     accel_x  = 0.0f;
    float     accel_y  = 0.0f;
    bool      imu_valid = false;
    Rpm       wheel_speed[WHEEL_COUNT];   // FL, FR, RL, RR (개별 휠속)
    float     vehicle_speed_mps  = 0.0f;  // 전륜 기준 추정 차속 (vehicle_speed 모듈)
    bool      vehicle_speed_valid = false;// false = 전륜 신호 불신 → TV 비활성
    float     pack_soc = 0.0f;    // 0..1, diagnostic BLE mirror only
    bool      pack_data_valid = false;
    float     pack_voltage_v = 0.0f;
    float     pack_current_a = 0.0f;
    int       pack_temperature_c = -40;
    uint32_t  bms_last_rx_ms = 0;
    uint16_t  gear_raw_adc = 0;
    Gear      gear = Gear::Neutral;
    // controller feedback (from CAN)
    ControllerFeedbackPart1 controller_fb1_L;
    ControllerFeedbackPart1 controller_fb1_R;
    ControllerFeedbackPart2 controller_fb2_L;
    ControllerFeedbackPart2 controller_fb2_R;
    uint32_t  controller_fb1_last_ms_L = 0;
    uint32_t  controller_fb1_last_ms_R = 0;
    uint32_t  controller_fb2_last_ms_L = 0;
    uint32_t  controller_fb2_last_ms_R = 0;
    bool      controller_feedback_fresh = false;
    bool      controller_fault_latched = false;
    // Cluster -> VCU command (from CAN_ID_CLUSTER_CMD, decoded in can_bus.cpp)
    // The Cluster labels its torque-vectoring request bit as "TC". In this
    // project that bit enables the existing TV pipeline; it is not a separate
    // wheel-slip traction-control algorithm.
    bool      tv_enable_requested = false;
    bool      regen_auto_requested = false;
    bool      paddock_requested = false;
    bool      paddock_active = false;
    bool      debug_requested = false;
    bool      cluster_cmd_alive = false;
    uint32_t  cluster_cmd_last_rx_ms = 0;
    // control outputs
    float     total_torque = 0.0f;   // signed A demand
    Amp       requested_torque_L;
    Amp       requested_torque_R;
    Amp       torque_L;              // motor phase-current command [A]
    Amp       torque_R;
    float     measured_bus_power_w = 0.0f;
    float     estimated_input_power_w = 0.0f;
    float     drive_limit_scale = 0.0f;
    bool      power_limited = false;
    bool      thermal_limited = false;
    // TV intermediate signals (관측/튜닝용; app_wiring이 TVOutput에서 복사)
    float     desired_yaw_rate = 0.0f;   // reference stage
    float     yaw_moment       = 0.0f;   // yaw_control stage (Mz)
    float     fz_L = 0.0f, fz_R = 0.0f;  // load stage (바퀴별 수직하중)
    float     max_torque_L = 0.0f, max_torque_R = 0.0f;  // traction stage
};

extern VehicleState state;   // defined in core/app_wiring.cpp
