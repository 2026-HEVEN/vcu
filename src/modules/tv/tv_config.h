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
    float tire_radius_m  = 0.2387f;  // 타이어 유효 구름반경 (실측 §1.1; 림 지름 아님)

    // --- 모터 / 구동계 (N·m ↔ A 변환용, §1.1 실측) ---
    float kt_nm_per_a    = 0.1266f;  // 모터 토크상수 [N·m/A]
    float gear_ratio     = 3.72f;    // 감속비 (모터축 → 바퀴축)

    // --- IMU 입력 단위 변환 (§2.2) ---
    float accel_to_mps2  = 9.80665f; // 드라이버가 g로 주는 ax/ay → m/s² (드라이버가 m/s²면 1.0)

    // --- 노면 / 타이어 ---
    float mu             = 1.0f;     // 노면 마찰계수 (지금은 상수; 추후 추정 확장 여지)

    // --- 레퍼런스 모델 (reference stage) ---
    float max_steer_rad  = 0.52f;    // steering Unit(±1) → 실제 조향각(rad) 매핑 (≈30°)
    float understeer_grad= 0.0f;     // 언더스티어 구배 K_us (0=중립)
    float desired_yaw_max= 60.0f;    // 목표 yaw rate 상한 (deg/s)
    float tv_min_speed_mps= 2.0f;    // 이 속도 미만이면 TV 전체 차단 (§4; ≈7km/h placeholder)

    // --- yaw 제어기 (yaw_control stage) PID ---
    float kp             = 0.0f;     // TODO: 튜닝
    float ki             = 0.0f;
    float kd             = 0.0f;
    float yaw_moment_max = 200.0f;   // Mz 출력 상한 (N·m 규약)
};

// 팀 공용 인스턴스. 위 기본값을 바꾸면 전체 파이프라인에 반영됩니다.
constexpr TVParams TV_PARAMS{};
