#pragma once
#include "types.h"
#include "modules/tv/traction.h"
#include "modules/tv/tv_config.h"
// [FILL-IN] Stage 5/5 — 토크 배분. 총전류 + Mz, 상한 제약 → 좌/우 전류 명령.

struct TVAllocOutput { Percent torque_L; Percent torque_R; };

// total_current : longitudinal이 준 부호 있는 총 구동 전류 요구 [A]
// yaw_moment    : yaw_control이 준 Mz [N·m] (좌우 차등의 근거)
// limit         : traction이 준 바퀴별 최대 허용 전류 [A]
// p             : 차량 제원 — Mz[N·m]→편도 전류차[A] 환산에 rw/track/kt/gear 사용 (§8)
// 반환           : 좌/우 전류 명령 (Percent — CAN 스케일, ±100 자동 clamp)
TVAllocOutput tv_alloc_compute(float total_current, float yaw_moment,
                               MaxTorque limit, const TVParams &p);
