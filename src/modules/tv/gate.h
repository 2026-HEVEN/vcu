#pragma once
#include "modules/tv/tv_config.h"
// Stage 0/5 — TV 활성 판정. 파이프라인 전체에서 이 함수만 게이트를 판단한다.
//
// 왜 별도 모듈인가: 이전에는 게이트 조건이 torque_vectoring.cpp(게인/차속),
// app_wiring.cpp(스위치/IMU/차속 유효성), reference.cpp(차속 재검사),
// can_bus.cpp(게인/차속 재계산) 네 곳에 흩어져 있었다. can_bus는 계기판에
// 보낼 차단 사유를 독립적으로 재계산했기 때문에, 한쪽만 고치면 "계기판은 ON
// 이라는데 실제로는 OFF"로 갈라질 수 있었다. 판정자를 하나로 고정한다.
//
// 이 게이트는 TV 파이프라인 자신의 판단만 다룬다. output_allowed(안전/구동
// 허가)와 test_override는 상위 관심사라 can_bus 쪽에 남는다.

struct TVInput;   // torque_vectoring.h — 순환 include 방지용 전방선언

// 개별 사유를 그대로 보존한다. car_check_status가 이 필드들을 TV_BLOCK_* 비트로
// 매핑하므로, 여기서 사유를 뭉뚱그리면 계기판 진단이 퇴화한다.
struct TVGate {
    bool active          = false;  // 다섯 조건 전부 충족
    bool driver_switch_on= false;  // 대시 TC/TV 스위치   -> TV_REQUEST_OFF
    bool gains_enabled   = false;  // kp/ki/kd 중 하나라도 0이 아님 -> TV_GAINS_ZERO
    bool imu_valid       = false;  // IMU 신선함          -> TV_IMU_INVALID
    bool speed_valid     = false;  // 전륜 차속 신뢰      -> TV_SPEED_INVALID
    bool speed_above_min = false;  // |v| >= tv_min_speed -> TV_LOW_SPEED
};

TVGate tv_gate_evaluate(const TVInput &in, const TVParams &p);
