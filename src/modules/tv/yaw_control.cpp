// [FILL-IN] Stage 2/5 — yaw 제어기   담당: ______
// NOTE(AI 구현): 이 함수는 Claude와 함께 작성함 — 담당자 검토 후 이 줄 삭제.
// 헤더(TVYawState{integral, prev_error})가 고정이라 표준 오차-미분 PID로 짰다.
// (팀원 참고코드처럼 "측정값 미분"으로 kick을 줄이려면 헤더에 필드 추가 논의 필요.)
#include "modules/tv/yaw_control.h"

namespace {
// ESP32(xtensa) 툴체인 libstdc++에 std::clamp(C++17)가 없어 직접 구현.
float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
}

// ── 이 함수가 하는 일 ─────────────────────────────────────────────
//   목표 yaw rate와 실측 yaw rate의 "오차"를 없애는 요 모멘트 Mz를 만든다.
//   Mz는 다음 단계(allocation)에서 좌우 토크 차등으로 바뀌어 차를 더/덜 돌린다.
float tv_yaw_compute(float desired_yaw, float measured_yaw, float dt,
                     const TVParams &p, TVYawState &s) {
    // dt가 비정상(0 이하)이면 적분/미분 모두 계산하지 않고 안전하게 0 반환.
    if (dt <= 0.0f) {
        return 0.0f;
    }

    float error = desired_yaw - measured_yaw;

    float p_term = p.kp * error;
    float d_term = p.kd * (error - s.prev_error) / dt;

    // 조건부 적분(anti-windup): 이번 적분까지 반영했을 때 상한을 넘고,
    // 그 방향으로 오차가 계속 미는 중이면 적분을 멈춘다.
    // → 포화 상태에서 오차가 계속 쌓여 나중에 반대로 확 풀리는 걸 막는다.
    float integral_candidate = s.integral + error * dt;
    float mz_candidate = p_term + p.ki * integral_candidate + d_term;
    bool saturating_high = (mz_candidate > p.yaw_moment_max) && (error > 0.0f);
    bool saturating_low  = (mz_candidate < -p.yaw_moment_max) && (error < 0.0f);
    if (!saturating_high && !saturating_low) {
        s.integral = integral_candidate;
    }

    float mz = p_term + p.ki * s.integral + d_term;
    s.prev_error = error;

    return clampf(mz, -p.yaw_moment_max, p.yaw_moment_max);
}
