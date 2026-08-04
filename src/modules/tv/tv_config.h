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
// NOTE(AI 구현): min_speed_mps / kt_nm_per_a / gear_ratio 세 필드는 Claude와 함께
// Stage 1/5 구현 중 추가함(기존 필드가 아니었음). 아래 각 필드에 "[실측 필요]"
// "[동적 식별 필요]" "[튜닝값]" "[설계 상수]" 태그로 어떤 값이 실차 없이는
// 확정 못 하는지 표시해뒀다.

struct TVParams {
    // --- 차량 제원 ---
    float mass_kg        = 300.0f;   // [실측 필요] 차량 총중량(운전자 포함) — 코너웨이트 4개 합
    float wheelbase_m    = 1.55f;    // [실측 필요] 축거 L — 줄자
    float track_m        = 1.20f;    // [실측 필요] 윤거(좌우 바퀴 간격) — 줄자
    float cg_height_m    = 0.30f;    // [동적 식별 필요] 무게중심 높이 h — 직접 측정 불가,
                                      //   롤 동역학 모델로 간접 추정(불확실도 큼). 검증 전엔
                                      //   Stage 3/4 결과를 과신하지 말 것.
    float weight_dist_r  = 0.50f;    // [실측 필요] 구동축(후) 정적 하중 배분 0..1 — 코너웨이트
    float tire_radius_m  = 0.26f;    // [실측 필요] 타이어 유효(구름) 반경 — 림 반지름 아님, 실측 필요

    // --- 노면 / 타이어 ---
    float mu             = 1.0f;     // [실측/보수적 가정 필요] 노면 마찰계수. 지금 값 1.0은
                                      //   낙관적 — 실측 전까지는 낮춰서(예 0.6~0.7) 안전마진 확보 권장.

    // --- 레퍼런스 모델 (reference stage) ---
    float max_steer_rad  = 0.52f;    // [캘리브레이션 필요] steering Unit(±1) → 실제 조향각(rad).
                                      //   포텐쇼미터 좌우 최대값을 실제 타이어각과 맞춰야 함.
    float understeer_grad= 0.0f;     // [동적 식별 필요] 언더스티어 구배 K_us. 슬라럼/원선회 시험 전엔 0(중립) 유지.
    float desired_yaw_max= 60.0f;    // [튜닝값] 목표 yaw rate 상한 (deg/s) — 설계 안전 상한, 실측 아님
    float min_speed_mps  = 1.0f;     // [튜닝값] 이 속도 미만이면 목표 yaw rate = 0

    // --- yaw 제어기 (yaw_control stage) PID ---
    float kp             = 0.0f;     // [실차 튜닝 필요] 0에서 시작해 발산 안 하는 선까지 서서히 증가
    float ki             = 0.0f;     // [실차 튜닝 필요] P 튜닝 끝난 뒤에 도입
    float kd             = 0.0f;     // [실차 튜닝 필요] 필요할 때만 마지막에 도입 (노이즈 민감)
    float yaw_moment_max = 200.0f;   // [튜닝값] Mz 출력 상한(N·m). 첫 실차 테스트는 100 이하 권장.

    // --- 구동계 (allocation stage: Mz[N·m] → 좌우 전류차[A] 환산) ---
    float kt_nm_per_a    = 0.1266f;  // [실측 필요] 모터 토크 상수 [N·m/A] — 팀원 문서 기재값, 우리 모터로 재검증 필요
    float gear_ratio     = 3.72f;    // [스펙 확인 필요] 감속비 — 팀원 문서 기재값, 감속기 스펙시트로 재확인
};

// 팀 공용 인스턴스. 위 기본값을 바꾸면 전체 파이프라인에 반영됩니다.
constexpr TVParams TV_PARAMS{};
