// [FILL-IN] Stage 5/5 — 토크 배분   담당: ______
#include "modules/tv/allocation.h"

// ── 이 함수가 하는 일 ─────────────────────────────────────────────
//   총토크를 좌우로 나눈다. 단, yaw_moment(Mz)만큼 좌우 차등을 주고,
//   각 바퀴의 트랙션 상한(limit)을 넘지 않게 제한한다. 파이프라인의 마지막 단.
//
// ── 구현 가이드 ──────────────────────────────────────────────────
//   float base = total_torque * 0.5f;                 // 기본 좌우 균등
//   float diff = yaw_moment / p.track_m / 2.0f;        // Mz → 좌우 토크차 (단위 규약 통일)
//   float tL = base - diff;                            // 부호는 좌표계에 맞춰
//   float tR = base + diff;
//   // 트랙션 상한 적용:
//   tL = clamp(tL, -limit.max_L, +limit.max_L);
//   tR = clamp(tR, -limit.max_R, +limit.max_R);
//   //  ★ 한쪽이 상한에 걸리면? 총토크 유지를 위해 반대쪽/전체를 스케일다운할지
//   //    (yaw 우선 vs 총량 우선) 정책을 팀에서 정할 것.
//   return { Percent(tL), Percent(tR) };               // Percent가 ±100 자동 clamp
//
// ── 주의 ─────────────────────────────────────────────────────────
//   * diff의 부호 규약을 reference/yaw/load와 하나로 통일(좌회전 + 등).
//   * track_m 등 제원이 필요하면 tv_config.h(TV_PARAMS)를 인자로 받도록 시그니처 확장
//     가능(코어 담당과 상의). 지금은 stub이라 미사용.
//
// ── SAFE STUB ────────────────────────────────────────────────────
//   50:50 (Mz·limit 무시) → 현재 차 거동과 완전히 동일.
TVAllocOutput tv_alloc_compute(float total_torque, float yaw_moment, MaxTorque limit) {
    (void)yaw_moment; (void)limit;
    float half = total_torque * 0.5f;
    return { Percent(half), Percent(half) };
}
