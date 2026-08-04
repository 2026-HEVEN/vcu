// [FILL-IN] Stage 3/5 — 하중 추정(모델 기반)   담당: ______
// NOTE(AI 구현): 이 함수는 Claude와 함께 작성함. mass_kg/cg_height_m/weight_dist_r가
// 전부 실측 전 placeholder라서, 계산 "구조"는 맞지만 수치는 아직 못 믿는다.
// 실측 이후 재검증 전까지는 낮은 yaw_moment_max/보수적 mu와 함께만 신뢰할 것.
#include "modules/tv/load.h"

// ── 이 함수가 하는 일 ─────────────────────────────────────────────
//   종/횡 가속으로 생기는 "하중 이동"을 계산해, 구동 두 바퀴의 수직하중 Fz를 추정한다.
//   Fz가 큰 바퀴일수록 노면을 세게 눌러 더 많은 종토크를 견딘다 → traction stage가 사용.
//
// ── 모델 ─────────────────────────────────────────────────────────
//   정적 하중(구동축 한쪽 바퀴):    Fz_static  = m*g*weight_dist_r / 2
//   종하중 이동(가속 시 구동축↑):   dFz_long   = m*ax*hCG / wheelbase   (좌우 동일 반영)
//   횡하중 이동(이 축의 몫만):      dFz_lat    = (m*ay*hCG / track) * weight_dist_r
//     ※ 앞뒤 롤강성 배분을 모르니, 이 축이 weight_dist_r만큼의 횡하중을 담당한다고
//       근사한다 — traction stage의 Fy 근사와 동일한 가정(일관성 유지).
//   좌표계(ISO 8855): ay>0(좌측 가속)이면 하중은 우측(바깥)으로 이동 → fz_R 증가.
WheelLoads tv_load_compute(float ax, float ay, const TVParams &p) {
    const float g = 9.80665f;
    float fz_static = p.mass_kg * g * p.weight_dist_r * 0.5f;

    float fz_long_delta = (p.mass_kg * ax * p.cg_height_m / p.wheelbase_m) * 0.5f;
    float fz_lat_delta = (p.mass_kg * ay * p.cg_height_m / p.track_m) * p.weight_dist_r * 0.5f;

    float fz_L = fz_static + fz_long_delta - fz_lat_delta;
    float fz_R = fz_static + fz_long_delta + fz_lat_delta;

    // 바퀴 들림(음수 하중) 방어 — 문서 불변식: Fz는 음수가 될 수 없다.
    if (fz_L < 0.0f) fz_L = 0.0f;
    if (fz_R < 0.0f) fz_R = 0.0f;

    return { fz_L, fz_R };
}
