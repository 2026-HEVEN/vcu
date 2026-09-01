#pragma once
#include "modules/realcar_calibration.h"
// ============================================================
//  토크벡터링 정책/튜닝 상수 — TV팀 공용 설정 파일
// ============================================================
//
// 5개 stage(reference/yaw/load/traction/allocation)가 모두 이 한 struct를 참조합니다.
// 차량 실측값은 realcar_calibration.h, 제어 정책/게인은 이 파일에서만 바꿉니다.
//
// Coordinate/unit contract (ISO 8855):
//   steering/yaw/Mz > 0 : left turn / counter-clockwise
//   ax_g > 0            : forward acceleration, rear load increases
//   ay_g > 0            : leftward acceleration, right wheel load increases
//   yaw rates           : deg/s, Mz: N*m, Fz: N, motor commands: phase A
// Confirm every sign on the stationary/low-current rig before enabling gains.

struct TVParams {
    // --- 차량 제원: realcar_calibration.h의 실차 시험 시작 프로파일 ---
    float mass_kg        = realcar_cal::provisional::VEHICLE_MASS_WITH_DRIVER_KG;
    float wheelbase_m    = realcar_cal::provisional::WHEELBASE_M;
    float track_m        = realcar_cal::provisional::REAR_TRACK_M;
    float cg_height_m    = realcar_cal::provisional::CG_HEIGHT_M;
    float weight_dist_r  = realcar_cal::provisional::REAR_STATIC_WEIGHT_FRACTION;
    float tire_radius_m  = realcar_cal::provisional::TV_FORCE_RADIUS_M;
    // Rear lateral-load-transfer distribution is not static weight distribution.
    float lltd_r          = realcar_cal::provisional::REAR_LLTD;

    // --- HPM05KW + 감속기: 모터 상전류[A] <-> 휠 종력 변환 ---
    float gear_ratio             = realcar_cal::confirmed::GEAR_RATIO;
    float motor_kt_nm_per_a      = realcar_cal::confirmed::MOTOR_KT_NM_PER_A;
    // Bring-up software ceiling. The separate 103 A value in confirmed is
    // the estimated continuous motor operating point, not this short test cap.
    float motor_current_max_a    =
        realcar_cal::bringup::DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A;

    // --- 노면 / 타이어 ---
    float mu             = 1.0f;     // 노면 마찰계수 (지금은 상수; 추후 추정 확장 여지)

    // --- 레퍼런스 모델 (reference stage) ---
    float max_steer_rad  = realcar_cal::provisional::MAX_ROAD_WHEEL_STEER_RAD;
    float understeer_grad= 0.0f;     // 언더스티어 구배 K_us (0=중립)
    float desired_yaw_max= 60.0f;    // 목표 yaw rate 상한 (deg/s)
    float tv_min_speed_mps= 1.0f;    // 이 속도 미만에서는 TV 차등 금지

    // --- yaw 제어기 (yaw_control stage) PID ---
    // 0/0/0은 master OFF다. 잭업·직선 검증 전에는 바꾸지 않는다.
    float kp             = 0.0f;
    float ki             = 0.0f;
    float kd             = 0.0f;
    float yaw_deadband_degps = 0.5f;
    float integral_max       = 100.0f; // integral-state hard limit [deg]
    float yaw_moment_max = 100.0f;   // Mz 출력 상한 [N·m]
};

// 팀 공용 인스턴스. 위 기본값을 바꾸면 전체 파이프라인에 반영됩니다.
constexpr TVParams TV_PARAMS{};
