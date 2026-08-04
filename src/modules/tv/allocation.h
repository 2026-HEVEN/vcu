#pragma once
#include "types.h"
#include "modules/tv/traction.h"
// [FILL-IN] Stage 5/5 — 토크 배분. 총토크 + Mz, 상한 제약 → 좌/우 토크 명령.

struct TVAllocOutput { Percent torque_L; Percent torque_R; }; // 실제 값은 상전류 [A]

// total_current_a : longitudinal이 준 부호 있는 총 상전류 요구 [A]
// yaw_moment_nm   : yaw_control이 준 Mz [N·m]
// limit           : traction이 준 모터별 상전류 한계 [A]
// 반환            : 좌/우 상전류 명령 [A] (Percent 레거시 타입, ±100 clamp)
TVAllocOutput tv_alloc_compute(float total_current_a, float yaw_moment_nm,
                               MaxTorque limit, const TVParams &p);
