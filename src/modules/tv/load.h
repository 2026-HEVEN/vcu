#pragma once
#include "types.h"
#include "modules/tv/tv_config.h"
// [FILL-IN] Stage 3/5 — 하중 추정(모델 기반). 가속도 → 바퀴별 수직하중 Fz.

struct WheelLoads { Newton fz_L; Newton fz_R; };   // 구동 두 바퀴의 수직하중 [N]

// ax_g : 종가속도 [g], +는 전방 가속(후축 하중 증가)
// ay_g : 횡가속도 [g], +는 좌향 가속(우측 하중 증가)
// 반환 : 좌/우 구동 바퀴 Fz [N]
WheelLoads tv_load_compute(GForce ax_g, GForce ay_g, const TVParams &p);
