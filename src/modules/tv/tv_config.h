#pragma once
#include "modules/realcar_calibration.h"
#include "modules/fixed_config.h"
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
    float gear_ratio             = fixed_config::vehicle::GEAR_RATIO;
    float motor_kt_nm_per_a      = fixed_config::vehicle::MOTOR_KT_NM_PER_A;
    // Bring-up software ceiling. The separate 103 A value in confirmed is
    // the estimated continuous motor operating point, not this short test cap.
    float motor_current_max_a    =
        realcar_cal::bringup::DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A;

    // --- 노면 / 타이어 ---
    // 노면 마찰계수 (지금은 상수). 목표 요 클램프 mu*g/v에도 쓰인다.
    // 10-01 원선회에서 TV OFF로 1.03~1.07 g가 나왔다. 1.0이면 한계 선회에서
    // 목표가 실측과 같아져 TV 차등이 약 2 A뿐이었다(1.1에서도 오차 5 deg/s).
    // 1.3: 한계 선회에서 목표가 실측보다 높게 나와 제어기가 차를 더 돌리게 한다.
    float mu             = 1.3f;
    // true: traction 한계(Stage 4)가 좌우 명령 자체를 자른다(= 총 구동력 삭감).
    // false: traction 한계는 차등(diff) 크기에만 쓰고, 총량은 모터 상한까지 50:50과 같다.
    // 10-01 원선회 로그에서 true일 때 ay 0.6 g에서 안쪽 후륜 한계 약 139 A로
    // 총 요청이 969 -> 291 A로 잘려 차속이 29 -> 18 km/h로 묶였다(모델상 1.0 g에서 0 A).
    // 마찰원 모델(mu, LLTD, CG 높이)을 실측으로 맞추기 전까지 false로 둔다.
    bool  traction_limits_total = false;

    // --- 레퍼런스 모델 (reference stage) ---
    float max_steer_rad  = realcar_cal::provisional::MAX_ROAD_WHEEL_STEER_RAD;
    // 언더스티어 구배 K_us [s^2]. 분모가 L + K_us*v^2 형태다.
    // 10-01 로그(센터 오프셋 제거 후) 선형 영역 적합값 약 0.0034 -> 0.003으로 시작.
    // 0이면 50 km/h에서 목표가 실측의 약 1.5배로 나와 Mz가 상시 포화된다.
    float understeer_grad= 0.003f;
    float desired_yaw_max= 100.0f;   // 목표 yaw rate 상한 (deg/s). 원선회 실측 약 72 deg/s라 60이면 목표가 실측보다 낮아진다.
    float tv_min_speed_mps= 1.0f;    // 이 속도 미만에서는 TV 차등 금지

    // --- yaw 제어기 (yaw_control stage) PID ---
    // 0/0/0은 master OFF다. 잭업·직선 검증 전에는 바꾸지 않는다.
    float kp             = 10.0f;
    float ki             = 0.0f;
    float kd             = 0.0f;
    float yaw_deadband_degps = 0.5f;
    float integral_max       = 100.0f; // integral-state hard limit [deg]
    // Mz 출력 상한 [N·m]. 10-01 슬라럼에서 40이면 TV ON 구간 80%가 포화(차등 ±18.6 A)라
    // ON/OFF 응답 차이가 없었다. 100 = 차등 약 ±47 A, 150 = 약 ±70 A.
    float yaw_moment_max = 150.0f;
    // 구동 중 안쪽 바퀴에 허용하는 회생(음) 상전류 상한 [A]. 0이면 부호 교차 금지.
    // 실제 허용량은 min(이 값, 총 요청/2)라 스로틀 0에서는 교차가 생기지 않는다.
    // 최종 게이트(app_wiring regen_allowed)는 TV 활성 + 좌우 부호 반대일 때만 통과시킨다.
    float inner_regen_max_a = 40.0f;
};

// 팀 공용 인스턴스. 위 기본값을 바꾸면 전체 파이프라인에 반영됩니다.
constexpr TVParams TV_PARAMS{};
