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
// ── 구현 (yaw 우선 제약 배분, 닫힌 해 3단계 — 문서 §8) ──────────
//   base(공통, 가감속) + diff(차등, 자세): I_L = base - ΔI, I_R = base + ΔI.
//   Mz[N·m] → 편도 전류차 ΔI[A] 환산: ΔI = Mz·rw/(track·kt·gear) (§8). diff>0 → 우측↑.
TVAllocOutput tv_alloc_compute(float total_current, float yaw_moment,
                               MaxTorque limit, const TVParams &p) {
    float base = total_current * 0.5f;                     // I_total/2 (공통 구동분)
    float denom = p.track_m * p.kt_nm_per_a * p.gear_ratio;
    float diff = (denom > 1e-9f) ? yaw_moment * p.tire_radius_m / denom : 0.0f;  // ΔI, diff>0→우측↑ (§2.1)
    float limL = limit.max_L, limR = limit.max_R;

    // 1) 차등을 실현 가능 범위로 먼저 제한 (yaw 우선). 순서 중요: 2단계 구간이 안 비게 보장.
    float d_max = (limL < limR) ? limL : limR;
    if (diff >  d_max) diff =  d_max;
    if (diff < -d_max) diff = -d_max;

    // 2) 남은 여유에서 공통분(base)이 가질 수 있는 구간 계산
    float a = -limL + diff, b = -limR - diff;
    float lo = (a > b) ? a : b;                 // max(-limL+diff, -limR-diff)
    float c =  limL + diff, e =  limR - diff;
    float hi = (c < e) ? c : e;                 // min( limL+diff,  limR-diff)

    // 3) base clamp. 0을 포함하는 구간이라 총토크 부호(회생/구동)가 뒤집히지 않음.
    if (base < lo) base = lo;
    if (base > hi) base = hi;

    float tL = base - diff;
    float tR = base + diff;
    return { Percent(tL), Percent(tR) };   // Percent가 ±100 자동 clamp (현재 A 스케일)
}
