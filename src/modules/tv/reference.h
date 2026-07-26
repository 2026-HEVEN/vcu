#pragma once
#include "types.h"
#include "modules/tv/tv_config.h"
// [FILL-IN] Stage 1/5 — 레퍼런스 모델. 조향 의도 → 목표 yaw rate(deg/s).

// steering  : Unit(-1..+1)  운전자 조향 입력 (부호 규약은 IMU yaw와 통일)
// speed_mps : float         추정 차속 [m/s] — 이미 환산된 값이 들어온다.
//                           어느 바퀴를 쓸지는 vehicle_speed 모듈이 이미 결정했다.
// 반환      : 목표 yaw rate [deg/s]
float tv_reference_compute(Unit steering, float speed_mps, const TVParams &p);
