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
// ── 구현 (이산 PID + 조건부 적분 anti-windup) ────────────────────
float tv_yaw_compute(float desired_yaw, float measured_yaw, float dt,
                     const TVParams &p, TVYawState &s) {
    float error = desired_yaw - measured_yaw;

    // 미분: 측정값 미분 −d(meas)/dt 로 derivative kick 완화 (§5). dt<=0 방어.
    //   setpoint(desired) 급변 시 error 미분은 스파이크를 만들지만 측정값 미분은 안 만든다.
    float deriv = (dt > 0.0f) ? -(measured_yaw - s.prev_measured) / dt : 0.0f;
    s.prev_measured = measured_yaw;

    // 적분 후보 (dt<=0이면 누적 없음)
    float integral_new = s.integral + ((dt > 0.0f) ? error * dt : 0.0f);
    float u = p.kp * error + p.ki * integral_new + p.kd * deriv;

    // 출력 포화 (±yaw_moment_max)
    const float lim = p.yaw_moment_max;
    float u_sat = u;
    if (u_sat >  lim) u_sat =  lim;
    if (u_sat < -lim) u_sat = -lim;

    // anti-windup(조건부 적분): 포화 중 & 오차가 포화를 더 미는 방향이면 적분을 얼린다(commit 안 함).
    //   출력은 u_sat 그대로 유지 — 저장하는 적분값만 동결해서 폭주를 막는다.
    bool saturated = (u != u_sat);
    bool pushing   = (u_sat > 0.0f && error > 0.0f) || (u_sat < 0.0f && error < 0.0f);
    s.integral = (saturated && pushing) ? s.integral : integral_new;

    return u_sat;   // 부호: error>0(더 좌회전 필요) → Mz>0(좌회전 보조) → 우측 바퀴 토크↑
}
