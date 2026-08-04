// [FILL-IN] Stage 1/5 — 레퍼런스 모델   담당: ______
#include "modules/tv/reference.h"

// ── 부호 규약 ───────────────────────────────────────────────────
//   좌회전 = yaw rate 양(+). steering도 좌회전 입력이 +가 되도록 통일한다.
//   (IMU/allocation과 반드시 같은 규약을 쓸 것 — Notion 문서 §2.1)
//
// ── 이 함수가 하는 일 / 구현 근거 ─────────────────────────────────
//   정상원선회 바이시클 모델(Notion 문서 §4):
//     delta   = steering * p.max_steer_rad                                   [rad]
//     desired = V * delta / (p.wheelbase_m + p.understeer_grad * V^2)        [rad/s]
//   rad/s → deg/s 변환 후 ±desired_yaw_max로 clamp(저속/큰 조향 발산 방지).
float tv_reference_compute(Unit steering, float speed_mps, const TVParams &p) {
    constexpr float RAD2DEG = 57.29577951308232f;

    float delta = (float)steering * p.max_steer_rad;
    float v = speed_mps > 0.0f ? speed_mps : 0.0f;   // 후진/음수 차속은 0으로 취급

    float denom = p.wheelbase_m + p.understeer_grad * v * v;
    float desired_rad_s = (denom > 1.0e-6f) ? (v * delta / denom) : 0.0f;

    float desired_deg_s = desired_rad_s * RAD2DEG;
    if (desired_deg_s > p.desired_yaw_max)  desired_deg_s = p.desired_yaw_max;
    if (desired_deg_s < -p.desired_yaw_max) desired_deg_s = -p.desired_yaw_max;
    return desired_deg_s;
}
