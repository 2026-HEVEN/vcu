// [FILL-IN] Stage 5/5 — 토크 배분   담당: ______
// NOTE(AI 구현): 이 함수는 Claude와 함께 작성함 — 담당자 검토 후 이 줄 삭제.
// allocation.h 시그니처에 const TVParams& 를 추가함(Mz→전류 환산에 track_m/
// kt_nm_per_a/gear_ratio/tire_radius_m이 필요해서). 그에 맞춰 orchestrator
// 호출부(torque_vectoring.cpp)도 함께 수정했다.
#include "modules/tv/allocation.h"

namespace {
// ESP32(xtensa) 툴체인 libstdc++에 std::clamp(C++17)가 없어 직접 구현.
float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
}

// ── 이 함수가 하는 일 ─────────────────────────────────────────────
//   총전류를 좌우로 나눈다. 단, yaw_moment(Mz)만큼 좌우 차등을 주고,
//   각 바퀴의 트랙션 상한(limit)을 넘지 않게 제한한다. 파이프라인의 마지막 단.
//
// ── Mz[N·m] → 좌우 전류차 diff[A] 유도 ────────────────────────────
//   Mz = (track/2) * (Fx_R - Fx_L)              // 요 모멘트 정의
//   Fx = torque / tire_radius = (I * kt * gear_ratio) / tire_radius
//   위 둘을 합치면: diff = I_R - I_total/2 = Mz * tire_radius / (track * kt * gear_ratio)
//
// ── 지금 정책: 좌우 "독립" clamp (임시) ────────────────────────────
//   diff를 먼저 구하고 base(=total/2) ± diff를 각자의 상한으로 자른다.
//   → 한쪽이 상한에 걸리면 그쪽만 깎이고 Mz가 원래 요구보다 덜 반영될 수 있다.
//   "총전류 우선 vs yaw 우선"은 아직 팀 결정이 안 된 부분이라 일부러 단순하게
//   시작했다. 실제로 어느 쪽이 안전한지는 다음 단계에서 같이 정하자.
TVAllocOutput tv_alloc_compute(float total_torque, float yaw_moment, MaxTorque limit,
                                const TVParams &p) {
    float denom = p.track_m * p.kt_nm_per_a * p.gear_ratio;
    float diff = (denom > 1e-4f) ? (yaw_moment * p.tire_radius_m) / denom : 0.0f;

    float base = total_torque * 0.5f;

    float raw_L = base - diff;
    float raw_R = base + diff;

    float clamped_L = clampf(raw_L, -limit.max_L, limit.max_L);
    float clamped_R = clampf(raw_R, -limit.max_R, limit.max_R);

    return { Percent(clamped_L), Percent(clamped_R) };
}
