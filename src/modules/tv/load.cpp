// [FILL-IN] Stage 3/5 — 하중 추정(모델 기반)   담당: ______
#include "modules/tv/load.h"

// ── 이 함수가 하는 일 ─────────────────────────────────────────────
//   종/횡 가속으로 생기는 "하중 이동"을 계산해, 구동 두 바퀴의 수직하중 Fz를 추정한다.
//   Fz가 큰 바퀴일수록 노면을 세게 눌러 더 많은 종토크를 견딘다 → traction stage가 사용.
//   (이게 "모델 기반 제어"의 핵심: 차량 동역학으로 바퀴별 접지력을 예측)
//
// ── 구현 가이드 ──────────────────────────────────────────────────
//   const float g = 9.81f;
//   정적 하중(구동축 한쪽 바퀴):
//       Fz_static = p.mass_kg * g * p.weight_dist_r / 2.0f;
//   횡하중 이동(바깥 바퀴로 이동):
//       dFz_lat = p.mass_kg * ay * p.cg_height_m / p.track_m;
//   종하중 이동(구동축으로/에서 이동 — 구동축 배분에 반영):
//       dFz_lon = p.mass_kg * ax * p.cg_height_m / p.wheelbase_m;   // 부호·배분 규약 결정
//   fz_L = Fz_static (±) dFz_lat/2 (+) 종이동분 ...   // 좌표계에 맞춰 부호 확정
//   fz_R = 반대쪽
//
// ── 주의 ─────────────────────────────────────────────────────────
//   * ax, ay의 단위(m/s^2)와 부호를 IMU 모듈과 반드시 맞출 것.
//   * Fz는 음수 불가(바퀴 들림) → 0으로 clamp.
//
// ── SAFE STUB ────────────────────────────────────────────────────
//   좌우 동일한 정적 하중만 반환(하중 이동 무시) → 좌우 대칭 → 현재 거동 유지.
WheelLoads tv_load_compute(float ax, float ay, const TVParams &p) {
    (void)ax; (void)ay;
    const float g = 9.81f;
    float fz_static = p.mass_kg * g * p.weight_dist_r * 0.5f;
    return { fz_static, fz_static };
}
