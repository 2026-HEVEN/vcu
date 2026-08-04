#pragma once
#include "modules/tv/tv_config.h"
#include "modules/tv/load.h"
// [FILL-IN] Stage 4/5 — 트랙션 한계(마찰원). 바퀴별 Fz → 바퀴별 최대 종토크.

struct MaxTorque { float max_L; float max_R; };   // 각 모터의 최대 상전류 [A]

// fz : 바퀴별 수직하중 (load stage 출력)
// ay_g : 횡가속도 [g] (횡력이 클수록 종방향 여유 grip 감소 — 마찰원)
// 반환 : 좌/우 최대 모터 상전류 [A]
MaxTorque tv_traction_compute(WheelLoads fz, float ay_g, const TVParams &p);
