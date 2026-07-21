#pragma once
#include "modules/tv/tv_config.h"
#include "modules/tv/load.h"
// [FILL-IN] Stage 4/5 — 트랙션 한계(마찰원). 바퀴별 Fz → 바퀴별 최대 종토크.

struct MaxTorque { float max_L; float max_R; };   // 각 바퀴가 낼 수 있는 최대 종토크

// fz : 바퀴별 수직하중 (load stage 출력)
// ay : 횡가속도 [m/s^2]  (횡력이 클수록 종방향 여유 grip 감소 — 마찰원)
// 반환 : 좌/우 최대 종토크 (단위는 allocation과 합의: A 또는 N·m)
MaxTorque tv_traction_compute(WheelLoads fz, float ay, const TVParams &p);
