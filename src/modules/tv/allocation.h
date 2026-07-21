#pragma once
#include "types.h"
#include "modules/tv/traction.h"
// [FILL-IN] Stage 5/5 — 토크 배분. 총토크 + Mz, 상한 제약 → 좌/우 토크 명령.

struct TVAllocOutput { Percent torque_L; Percent torque_R; };

// total_torque : longitudinal이 준 부호 있는 총 토크 요구
// yaw_moment   : yaw_control이 준 Mz (좌우 차등의 근거)
// limit        : traction이 준 바퀴별 최대 종토크
// 반환          : 좌/우 토크 명령 (Percent — CAN 스케일, ±100 자동 clamp)
TVAllocOutput tv_alloc_compute(float total_torque, float yaw_moment, MaxTorque limit);
