// [FILL-IN] Stage 2/5 — yaw 제어기   담당: ______
#include "modules/tv/yaw_control.h"

// ── 이 함수가 하는 일 ─────────────────────────────────────────────
//   목표 yaw rate와 실측 yaw rate의 오차를 없애는 요 모멘트 Mz를 PID로 만든다.
//   출력은 항상 ±yaw_moment_max로 clamp(Notion 문서 §5 "Mz는 항상 안전 상한 이하").
//
// ── anti-windup ────────────────────────────────────────────────
//   조건부 적분: 출력이 이미 포화된 방향으로 오차가 더 쌓이면 적분을 멈추고,
//   반대 방향 오차(포화를 빠져나오는 방향)는 정상적으로 적분한다.
//   (Notion 문서 §5: "출력 포화 중 같은 방향의 오차가 남으면 조건부 적분으로 windup을 막는다")
float tv_yaw_compute(float desired_yaw, float measured_yaw, float dt,
                     const TVParams &p, TVYawState &s) {
    float error = desired_yaw - measured_yaw;

    if (dt <= 0.0f) {
        // 비정상 tick — 적분 진행하지 않고 이전 이력 기준 P항만 반영
        float mz = p.kp * error;
        if (mz > p.yaw_moment_max)  mz = p.yaw_moment_max;
        if (mz < -p.yaw_moment_max) mz = -p.yaw_moment_max;
        return mz;
    }

    float integral_candidate = s.integral + error * dt;
    float deriv = (error - s.prev_error) / dt;
    float mz_unclamped = p.kp * error + p.ki * integral_candidate + p.kd * deriv;

    bool saturated_same_direction =
        (mz_unclamped > p.yaw_moment_max  && error > 0.0f) ||
        (mz_unclamped < -p.yaw_moment_max && error < 0.0f);
    if (!saturated_same_direction) {
        s.integral = integral_candidate;
    }
    s.prev_error = error;

    float mz = p.kp * error + p.ki * s.integral + p.kd * deriv;
    if (mz > p.yaw_moment_max)  mz = p.yaw_moment_max;
    if (mz < -p.yaw_moment_max) mz = -p.yaw_moment_max;
    return mz;
}
