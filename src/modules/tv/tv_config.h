#pragma once
// ============================================================
//  [FILL-IN] 토크벡터링 튜닝 상수 — TV팀 공용 설정 파일
//  이 파일은 코어(LOCKED)가 아니다. 실제 차량 값으로 자유롭게 수정하세요.
//  단, 여기 있는 값은 지금 전부 "임시 placeholder"입니다. 실측/스펙으로 교체 필요.
// ============================================================
//
// 5개 stage(reference/yaw/load/traction/allocation)가 모두 이 한 struct를 참조합니다.
// 튜닝은 "여기 한 곳"에서만 — 각 .cpp에 상수를 흩뿌리지 마세요.

struct TVParams {
    // --- 차량 제원 (TODO: 실측값으로 교체) ---
    float mass_kg        = 300.0f;   // 차량 총중량 (운전자 포함)
    float wheelbase_m    = 1.55f;    // 축거 L
    float track_m        = 1.20f;    // 윤거 (좌우 바퀴 간격)
    float cg_height_m    = 0.30f;    // 무게중심 높이 h
    float weight_dist_r  = 0.50f;    // 구동축(후) 정적 하중 배분 0..1
    float tire_radius_m  = 0.2387f;  // 구름반경 (Notion §1 계산 예시값 — 아직 최종 실측 확정 아님, TODO 재확인)

    // --- 구동계 (Notion §1 TV 권한 계산 실측값) ---
    float kt_nm_per_a     = 0.1266f; // 모터 토크상수 [N·m/A] (다이노 실측, 상전류 RMS 기준)
    float gear_ratio      = 3.72f;   // 감속비 (확정)
    // 바퀴당 구동계 상전류 상한 [A]. 모터 연속정격 13 N·m ÷ kt 로 산출.
    // traction stage가 그립 기반 상한과 이 값을 같이 clamp해야 한다 —
    // 그립만 보면 모터가 못 버티는 전류를 상한으로 낼 수 있음(피크는 300A/10초).
    float motor_current_max_a = 103.0f;

    // --- 센서 단위 변환 ---
    // IMU 드라이버가 ax/ay를 g 단위로 준다(Notion §2.2). load/traction stage
    // 진입 시 이 값을 곱해 m/s^2로 바꾼다. 드라이버 단위가 바뀌면 여기만 고치면 됨.
    float accel_to_mps2  = 9.80665f;

    // --- 노면 / 타이어 ---
    float mu             = 1.0f;     // 노면 마찰계수 (지금은 상수; 추후 추정 확장 여지)

    // --- 레퍼런스 모델 (reference stage) ---
    float max_steer_rad  = 0.52f;    // steering Unit(±1) → 실제 조향각(rad) 매핑 (≈30°)
    float understeer_grad= 0.0f;     // 언더스티어 구배 K_us÷g 형태, s^2/m 단위 (0=중립)
    float desired_yaw_max= 60.0f;    // 목표 yaw rate 소프트웨어 안전상한 (deg/s). 마찰 한계(μg/v)와 함께 min 적용.
    // TV 활성 최저 차속 [m/s]. 이 아래에서는 reference stage가 0을 반환해 TV를 끈다
    // (저속에서 yaw rate 추정/기준 자체가 불안정하므로).
    float tv_min_speed_mps = 1.0f;

    // --- yaw 제어기 (yaw_control stage) PID ---
    float kp             = 0.0f;     // TODO: 튜닝
    float ki             = 0.0f;
    float kd             = 0.0f;
    // Mz 출력 상한 (N·m 규약). 타이어 그립 한계 요모멘트는 약 1140 N·m(Notion §1)지만,
    // 검증 전 미튜닝 상태의 안전 상한으로 100 N·m만 허용한다(TV 권한 약 10%).
    float yaw_moment_max = 100.0f;
};

// 팀 공용 인스턴스. 위 기본값을 바꾸면 전체 파이프라인에 반영됩니다.
constexpr TVParams TV_PARAMS{};
