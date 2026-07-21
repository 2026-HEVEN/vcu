// [FILL-IN] Stage 1/5 — 레퍼런스 모델   담당: ______
#include "modules/tv/reference.h"

// ── 이 함수가 하는 일 ─────────────────────────────────────────────
//   운전자의 "조향 의도"를, 차가 이상적으로 내야 할 "목표 yaw rate"로 바꾼다.
//   다음 단계(yaw_control)가 이 목표와 IMU 실측 yaw rate를 비교해 오차를 만든다.
//
// ── 구현 가이드 (정상원선회 바이시클 모델) ────────────────────────
//   1) 조향각 환산:   delta = (float)steering * p.max_steer_rad          [rad]
//   2) 차속 환산:     V = (float)speed  → m/s  (Rpm×타이어반경 등, 단위 확인!)
//   3) 목표 yaw:      desired = V * delta / (p.wheelbase_m + p.understeer_grad * V*V)  [rad/s]
//   4) 단위 통일:     rad/s → deg/s  (IMU yaw_rate가 deg/s이므로 맞춘다)
//   5) 상한:          clamp(±p.desired_yaw_max)  — 저속/큰 조향에서 발산 방지
//
// ── 주의 ─────────────────────────────────────────────────────────
//   * 부호 규약(좌회전이 +인지)을 IMU 모듈·allocation과 반드시 통일할 것.
//   * V가 0 근처일 때 0으로 나누지 않게 처리.
//
// ── SAFE STUB ────────────────────────────────────────────────────
//   목표 yaw = 0  → yaw 모멘트 요청 없음 → 현재 차 거동과 100% 동일.
float tv_reference_compute(Unit steering, Rpm speed, const TVParams &p) {
    (void)steering; (void)speed; (void)p;
    return 0.0f;
}
