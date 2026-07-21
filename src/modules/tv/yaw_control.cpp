// [FILL-IN] Stage 2/5 — yaw 제어기   담당: ______
#include "modules/tv/yaw_control.h"

// ── 이 함수가 하는 일 ─────────────────────────────────────────────
//   목표 yaw rate와 실측 yaw rate의 "오차"를 없애는 요 모멘트 Mz를 만든다.
//   Mz는 다음 단계(allocation)에서 좌우 토크 차등으로 바뀌어 차를 더/덜 돌린다.
//
// ── 구현 가이드 (PID + 필요시 피드포워드) ─────────────────────────
//   float error = desired_yaw - measured_yaw;
//   s.integral += error * dt;                          // 적분
//   // ★ 적분 와인드업 방지: s.integral을 일정 범위로 clamp 하거나 포화 시 정지
//   float deriv = (dt > 0.0f) ? (error - s.prev_error) / dt : 0.0f;
//   s.prev_error = error;
//   float Mz = p.kp*error + p.ki*s.integral + p.kd*deriv;
//   return clamp(Mz, -p.yaw_moment_max, +p.yaw_moment_max);
//
// ── 주의 ─────────────────────────────────────────────────────────
//   * s(TVYawState)에만 이력을 저장 — 전역 static 금지(테스트 오염).
//   * 게인 p.kp/ki/kd는 tv_config.h에서 튜닝. 여기 상수 박지 말 것.
//   * 와인드업 방지는 선택이 아니라 필수 (긴 정상상태 오차에서 폭주함).
//
// ── SAFE STUB ────────────────────────────────────────────────────
//   Mz = 0  → 좌우 차등 없음 → 현재 차 거동과 100% 동일.
float tv_yaw_compute(float desired_yaw, float measured_yaw, float dt,
                     const TVParams &p, TVYawState &s) {
    (void)desired_yaw; (void)measured_yaw; (void)dt; (void)p; (void)s;
    return 0.0f;
}
