#pragma once
#include "modules/tv/tv_config.h"
// [FILL-IN] Stage 3/5 — 하중 추정(모델 기반). 가속도 → 바퀴별 수직하중 Fz.

struct WheelLoads { float fz_L; float fz_R; };   // 구동 두 바퀴의 수직하중 [N]

// ax : 종가속도 [m/s^2]   (IMU, 부호 규약 통일)
// ay : 횡가속도 [m/s^2]
// 반환 : 좌/우 구동 바퀴 Fz [N]
WheelLoads tv_load_compute(float ax, float ay, const TVParams &p);
