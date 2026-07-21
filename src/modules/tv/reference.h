#pragma once
#include "types.h"
#include "modules/tv/tv_config.h"
// [FILL-IN] Stage 1/5 — 레퍼런스 모델. 조향 의도 → 목표 yaw rate(deg/s).

// steering : Unit(-1..+1)  운전자 조향 입력 (부호 규약은 IMU yaw와 통일)
// speed    : Rpm           휠 속도 (차속 환산용)
// 반환      : 목표 yaw rate [deg/s]
float tv_reference_compute(Unit steering, Rpm speed, const TVParams &p);
