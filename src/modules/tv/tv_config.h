#pragma once
// ============================================================
//  [FILL-IN] 토크벡터링 튜닝 상수 — TV팀 공용 설정 파일
//  이 파일은 코어(LOCKED)가 아니다. 실제 차량 값으로 자유롭게 수정하세요.
//  단, 여기 있는 값은 지금 전부 "임시 placeholder"입니다. 실측/스펙으로 교체 필요.
// ============================================================
//
// 5개 stage(reference/yaw/load/traction/allocation)가 모두 이 한 struct를 참조합니다.
// 튜닝은 "여기 한 곳"에서만 — 각 .cpp에 상수를 흩뿌리지 마세요.
//
// Coordinate/unit contract (ISO 8855):
//   steering/yaw/Mz > 0 : left turn / counter-clockwise
//   ax_g > 0            : forward acceleration, rear load increases
//   ay_g > 0            : leftward acceleration, right wheel load increases
//   yaw rates           : deg/s, Mz: N*m, Fz: N, motor commands: phase A
// Confirm every sign on the stationary/low-current rig before enabling gains.

struct TVParams {
    // --- 차량 제원 (TODO: 실측값으로 교체) ---
    float mass_kg        = 300.0f;   // TODO: corner-weight measurement
    float wheelbase_m    = 1.55f;    // TODO: vehicle measurement
    float track_m        = 1.20f;    // TODO: vehicle measurement
    float cg_height_m    = 0.30f;    // TODO: dynamic identification
    float weight_dist_r  = 0.50f;    // TODO: rear static weight fraction
    float tire_radius_m  = 0.2387f;  // measured 1.50 m rolling circumference / 2pi
    // Rear lateral-load-transfer distribution is not static weight distribution.
    float lltd_r          = 0.50f;    // TODO: suspension/roll-stiffness identification

    // --- HPM05KW + 감속기: 모터 상전류[A] <-> 휠 종력 변환 ---
    float gear_ratio             = 3.72f;   // confirmed reduction ratio
    float motor_kt_nm_per_a      = 0.1266f; // HPM05KW@48V dyno regression
    // HPM05KW continuous torque 13 N·m / measured Kt 0.1266 ~= 103 A.
    // Peak 300 A remains representable by Amp but is not enabled here.
    float motor_current_max_a    = 103.0f;

    // --- 노면 / 타이어 ---
    float mu             = 1.0f;     // 노면 마찰계수 (지금은 상수; 추후 추정 확장 여지)

    // --- 레퍼런스 모델 (reference stage) ---
    float max_steer_rad  = 0.52f;    // steering Unit(±1) → 실제 조향각(rad) 매핑 (≈30°)
    float understeer_grad= 0.0f;     // 언더스티어 구배 K_us (0=중립)
    float desired_yaw_max= 60.0f;    // 목표 yaw rate 상한 (deg/s)
    float tv_min_speed_mps= 1.0f;    // 이 속도 미만에서는 TV 차등 금지

    // --- yaw 제어기 (yaw_control stage) PID ---
    float kp             = 0.0f;     // TODO: 튜닝
    float ki             = 0.0f;
    float kd             = 0.0f;
    float yaw_deadband_degps = 0.5f;
    float integral_max       = 100.0f; // integral-state hard limit [deg]
    float yaw_moment_max = 100.0f;   // Mz 출력 상한 [N·m]
};

// 팀 공용 인스턴스. 위 기본값을 바꾸면 전체 파이프라인에 반영됩니다.
constexpr TVParams TV_PARAMS{};
