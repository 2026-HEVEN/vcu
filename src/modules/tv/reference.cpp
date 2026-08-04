// [FILL-IN] Stage 1/5 — 레퍼런스 모델   담당: ______
#include "modules/tv/reference.h"

// ── 이 함수가 하는 일 ─────────────────────────────────────────────
//   운전자의 "조향 의도"를, 차가 이상적으로 내야 할 "목표 yaw rate"로 바꾼다.
//   다음 단계(yaw_control)가 이 목표와 IMU 실측 yaw rate를 비교해 오차를 만든다.
//
// ── 구현 가이드 (정상원선회 바이시클 모델) ────────────────────────
//   1) 조향각 환산:   delta = (float)steering * p.max_steer_rad          [rad]
//   2) 차속:          V = speed_mps  (환산 불필요 — vehicle_speed 모듈이 이미 함)
//   3) 목표 yaw:      desired = V * delta / (p.wheelbase_m + p.understeer_grad * V*V)  [rad/s]
//   4) 단위 통일:     rad/s → deg/s  (IMU yaw_rate가 deg/s이므로 맞춘다)
//   5) 상한:          clamp(±p.desired_yaw_max)  — 저속/큰 조향에서 발산 방지
//
// ── 주의 ─────────────────────────────────────────────────────────
//   * 부호 규약(좌회전이 +인지)을 IMU 모듈·allocation과 반드시 통일할 것.
//   * V가 0 근처일 때 0으로 나누지 않게 처리.
//
// ── 구현 ─────────────────────────────────────────────────────────
//   정상원선회 바이시클 모델 + 마찰 한계 클램프.
float tv_reference_compute(Unit steering, float speed_mps, const TVParams &p) {
    constexpr float RAD2DEG = 57.29578f;   // 180/pi

    float delta = (float)steering * p.max_steer_rad;   // Unit(±1) → 실제 조향각 [rad]
    float V = speed_mps > 0.0f ? speed_mps : 0.0f;     // 음수 차속 방어

    // 저속 컷오프(§4): 임계 미만이면 목표 0. 저속에서 IMU 노이즈·μg/V 발산 회피.
    if (V < p.tv_min_speed_mps) return 0.0f;

    // 목표 yaw rate [rad/s] = V·δ / (L + K_us·V²)   (0 나눗셈 방어)
    float denom = p.wheelbase_m + p.understeer_grad * V * V;
    if (denom < 1e-6f) denom = 1e-6f;
    float r_deg = (V * delta / denom) * RAD2DEG;       // → deg/s (IMU와 단위 통일)

    // 상한 = min(설정 상한, 마찰 한계 r_max=μg/V).  마찰 한계 없으면 도달 불가 목표 → 와인드업.
    float limit = p.desired_yaw_max;
    if (V > 0.1f) {
        float r_fric = (p.mu * 9.81f / V) * RAD2DEG;
        if (r_fric < limit) limit = r_fric;
    }
    if (r_deg >  limit) r_deg =  limit;
    if (r_deg < -limit) r_deg = -limit;
    return r_deg;   // 부호: steering + = 좌회전 = yaw +  (IMU 규약과 일치)
}
