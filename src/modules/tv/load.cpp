// [FILL-IN] Stage 3/5 — 하중 추정(모델 기반)   담당: ______
#include "modules/tv/load.h"

// ── 부호 규약(Notion 문서 §2.1/§6) ────────────────────────────────
//   ay > 0 (좌회전) → 하중은 바깥쪽인 우측으로 이동 → fz_R 증가, fz_L 감소.
//   ax > 0 (전방가속) → 구동축(후축) 하중 증가 → 이 축의 좌우 Fz가 함께 증가.
//
// ── 구현 근거 ──────────────────────────────────────────────────
//   정적(구동축 한쪽): Fz_static     = m*g*weight_dist_r / 2
//   횡하중 이동(축 전체): dFz_lat    = m*ay*cg_height_m / track_m   (Notion §6 ΔFz,total)
//   종하중 이동(축 전체): dFz_lon    = m*ax*cg_height_m / wheelbase_m (Notion §3 ΔFz,long,total)
//   fz_L = Fz_static - dFz_lat/2 + dFz_lon/2
//   fz_R = Fz_static + dFz_lat/2 + dFz_lon/2
//   Fz는 음수(바퀴 들림) 불가 → 0으로 clamp(Notion §3.10 불변식).
WheelLoads tv_load_compute(float ax, float ay, const TVParams &p) {
    const float g = 9.81f;

    float fz_static = p.mass_kg * g * p.weight_dist_r * 0.5f;
    float dfz_lat = p.mass_kg * ay * p.cg_height_m / p.track_m;
    float dfz_lon = p.mass_kg * ax * p.cg_height_m / p.wheelbase_m;

    float fz_L = fz_static - dfz_lat * 0.5f + dfz_lon * 0.5f;
    float fz_R = fz_static + dfz_lat * 0.5f + dfz_lon * 0.5f;

    if (fz_L < 0.0f) fz_L = 0.0f;
    if (fz_R < 0.0f) fz_R = 0.0f;

    return { fz_L, fz_R };
}
