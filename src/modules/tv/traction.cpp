// [FILL-IN] Stage 4/5 — 트랙션 한계(마찰원)   담당: ______
#include "modules/tv/traction.h"
#include <cmath>

// ── 이 함수가 하는 일 / 구현 근거(Notion 문서 §7) ─────────────────
//   마찰원: sqrt(Fx^2+Fy^2) ≤ μ*Fz  →  Fx,max = sqrt(max(0, (μ*Fz)^2 - Fy_est^2))
//   전류 환산:  current_max = Fx,max * tire_radius_m / (kt * gear_ratio)
//   Fy_est는 정밀 모델이 없으므로, 이 축(구동축)이 부담하는 횡력을
//   질량배분(weight_dist_r)으로 근사하고 좌우 균등하다고 가정한다
//   (Notion §7: "Fy_est가 없거나 Fz 신뢰도가 낮으면 보수적인 μ와 근사치로 제한").
static float wheel_current_limit(float fz_wheel, float fy_per_wheel, const TVParams &p) {
    float f_mu = p.mu * fz_wheel;
    float remain_sq = f_mu * f_mu - fy_per_wheel * fy_per_wheel;
    if (remain_sq < 0.0f) remain_sq = 0.0f;   // 횡한계 초과 → sqrt 음수 방지, 종 여유 0
    float fx_max = sqrtf(remain_sq);
    return fx_max * p.tire_radius_m / (p.kt * p.gear_ratio);
}

MaxTorque tv_traction_compute(WheelLoads fz, float ay, const TVParams &p) {
    float fy_axle = p.mass_kg * ay * p.weight_dist_r;
    float fy_per_wheel = fabsf(fy_axle) * 0.5f;

    return { wheel_current_limit(fz.fz_L, fy_per_wheel, p),
             wheel_current_limit(fz.fz_R, fy_per_wheel, p) };
}
