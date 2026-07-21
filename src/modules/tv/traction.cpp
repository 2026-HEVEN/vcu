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
// ── SAFE STUB ────────────────────────────────────────────────────
//   사실상 무제한(큰 값) 반환 → allocation이 상한에 안 걸림 → 현재 거동(무제한) 유지.
MaxTorque tv_traction_compute(WheelLoads fz, float ay, const TVParams &p) {
    (void)fz; (void)ay; (void)p;
    constexpr float UNLIMITED = 1.0e6f;
    return { UNLIMITED, UNLIMITED };
}
