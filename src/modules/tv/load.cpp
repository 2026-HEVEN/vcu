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
// ── 구현 (정적 + 종/횡 하중이동, Fz 0 clamp) ─────────────────────
//   부호 규약(§2.1): ay+(좌회전) → 하중 우측(바깥) 이동, ax+(가속) → 후축(구동) 하중 증가.
WheelLoads tv_load_compute(float ax, float ay, const TVParams &p) {
    const float g = 9.81f;
    float m_axle = p.mass_kg * p.weight_dist_r;        // 구동축(후)이 지지하는 질량

    // 정적 하중 (바퀴당)
    float fz_static = m_axle * g * 0.5f;

    // 종하중 이동: 가속 시 후축(구동축)으로 하중 증가. 바퀴당 절반.
    float dFz_lon = (p.mass_kg * ax * p.cg_height_m / p.wheelbase_m) * 0.5f;

    // 횡하중 이동: 구동축 단위 모멘트 평형. ay+ → 우측(바깥) 증가 / 좌측(안쪽) 감소.
    float dFz_lat = m_axle * ay * p.cg_height_m / p.track_m;

    float fz_L = fz_static + dFz_lon - dFz_lat;
    float fz_R = fz_static + dFz_lon + dFz_lat;

    // 바퀴 들림(음수) 방지 → 다음 stage의 sqrt(Fz)가 깨지지 않게 0 clamp
    if (fz_L < 0.0f) fz_L = 0.0f;
    if (fz_R < 0.0f) fz_R = 0.0f;
    return { fz_L, fz_R };
}
