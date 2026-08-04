#pragma once
#include "types.h"
#include "modules/tv/traction.h"
// [FILL-IN] Stage 5/5 — 토크 배분. 총토크 + Mz, 상한 제약 → 좌/우 토크 명령.
// NOTE(AI 수정): const TVParams& p 파라미터를 Claude와 함께 추가함(원래는 없었음).
// Mz[N·m]→전류[A] 환산에 track_m/kt_nm_per_a/gear_ratio가 필요해서 확장.
// 팀 문서상 "필요하면 시그니처 확장 가능(코어 담당과 상의)"이라고 이미 허용돼 있던 부분.

struct TVAllocOutput { Percent torque_L; Percent torque_R; };

// total_torque : longitudinal이 준 부호 있는 총 전류 요구 [A] (state.h 주석 기준)
// yaw_moment   : yaw_control이 준 Mz [N·m] (좌우 차등의 근거)
// limit        : traction이 준 바퀴별 최대 전류 [A]
// p            : track_m/kt_nm_per_a/gear_ratio/tire_radius_m 등 Mz→전류 환산에 필요
// 반환          : 좌/우 전류 명령 (Percent — CAN 스케일, ±100A 자동 clamp)
TVAllocOutput tv_alloc_compute(float total_torque, float yaw_moment, MaxTorque limit,
                                const TVParams &p);
