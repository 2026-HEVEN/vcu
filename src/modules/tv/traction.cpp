// [FILL-IN] Stage 4/5 — 트랙션 한계(마찰원)   담당: ______
#include "modules/tv/traction.h"
#include <cmath>

// ── 이 함수가 하는 일 ─────────────────────────────────────────────
//   각 바퀴가 미끄러지지 않고 낼 수 있는 "최대 종토크"를 Fz와 노면 μ로 계산한다.
//   allocation stage가 이 상한을 넘지 않게 좌우 토크를 배분한다(휠스핀/그립상실 방지).
//
// ── 구현 가이드 (마찰원, friction circle) ─────────────────────────
//   최대 마찰력:      F_mu = p.mu * Fz                       // 바퀴별
//   횡력 사용분:      Fy   ≈ (그 바퀴가 부담하는 횡력)        // ay·질량배분으로 근사
//   종방향 여유:      Fx_max = sqrt( max(0, F_mu^2 - Fy^2) )  // 횡을 쓸수록 종 여유↓
//   최대 종토크:      T_max = Fx_max * p.tire_radius_m        // 기어비 있으면 반영
//   좌/우 각각 계산해서 반환.
//
// ── 주의 ─────────────────────────────────────────────────────────
//   * 단위를 allocation과 통일(A로 낼지 N·m로 낼지 팀에서 먼저 합의).
//   * F_mu^2 - Fy^2 < 0 (횡한계 초과) 케이스를 0으로 막을 것(sqrt 음수 방지).
//
// ── 구현 (마찰원 friction circle) ────────────────────────────────
//   단위 주의: 출력은 N·m. tv_config.h에 kt/gear가 없어 A로 못 바꾼다.
//   A 변환(I = T/(kt·gear))은 kt·gear 파라미터가 추가되면(amp-type) allocation과 함께 확정.
MaxTorque tv_traction_compute(WheelLoads fz, float ay, const TVParams &p) {
    float m_axle = p.mass_kg * p.weight_dist_r;
    float Fy_axle = m_axle * ay;                       // 구동축이 부담하는 총 횡력 (근사)
    float fz_sum  = fz.fz_L + fz.fz_R;

    // 한 바퀴의 최대 종토크 [N·m]
    auto wheel_max = [&](float fz_wheel) -> float {
        float Fmu = p.mu * fz_wheel;                                    // 마찰원 반경 [N]
        float Fy  = (fz_sum > 1e-6f) ? Fy_axle * (fz_wheel / fz_sum) : 0.0f;  // 하중비 횡력 배분
        float inside = Fmu * Fmu - Fy * Fy;                             // 종방향 여유²
        float Fx_max = (inside > 0.0f) ? sqrtf(inside) : 0.0f;          // 횡한계 초과 → 0 (sqrt 음수 방지)
        return Fx_max * p.tire_radius_m;                                // 힘 → 토크 [N·m]
    };
    return { wheel_max(fz.fz_L), wheel_max(fz.fz_R) };
}
