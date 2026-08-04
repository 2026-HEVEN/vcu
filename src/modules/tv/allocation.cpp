// [FILL-IN] Stage 5/5 — 토크 배분   담당: ______
#include "modules/tv/allocation.h"

// ── 이 함수가 하는 일 / 구현 근거(Notion 문서 §8) ─────────────────
//   Mz = (track/2)*(Fx_R - Fx_L) 를 전류 차등으로 역산:
//     dI = Mz * tire_radius_m / (track_m * kt * gear_ratio)
//   I_R = total/2 + dI,  I_L = total/2 - dI
//   (양의 Mz는 우측 전류를 좌측보다 크게 만든다 — Notion §8 불변식)
//
// ── 트랙션 상한 적용 ────────────────────────────────────────────
//   TODO(팀 결정, Notion §8 "팀이 정할 것"): 한쪽이 상한(limit)에 걸렸을 때
//   총토크 유지 vs yaw(Mz) 유지 중 우선순위가 아직 미정. 지금은 좌/우 독립
//   clamp만 적용한다 — 포화 시 요청한 Mz가 그대로 실현되지 않을 수 있다.
TVAllocOutput tv_alloc_compute(float total_torque, float yaw_moment, MaxTorque limit) {
    const TVParams &p = TV_PARAMS;

    float base = total_torque * 0.5f;
    float dI = yaw_moment * p.tire_radius_m / (p.track_m * p.kt * p.gear_ratio);

    float tL = base - dI;
    float tR = base + dI;

    if (tL > limit.max_L)  tL = limit.max_L;
    if (tL < -limit.max_L) tL = -limit.max_L;
    if (tR > limit.max_R)  tR = limit.max_R;
    if (tR < -limit.max_R) tR = -limit.max_R;

    return { Percent(tL), Percent(tR) };
}
