// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include <cstdint>
#include "modules/gear.h"

// [LOCKED] 좌·우 EZkontrol 명령 확정 계층. 순수 전이 로직은 호스트에서 테스트한다.
// 구현은 src/logic/motor_command.cpp. 하드웨어와 전역 state를 모른다.
//
// 문제: core 0(스케줄러 100 Hz)이 쓰는 명령값을 core 1(CAN life 태스크 20 Hz)이
// 필드 단위로 따로 읽어, 좌우가 서로 다른 tick의 값이 되거나 기어가 두 번 따로
// 읽혀 좌우 목표 rpm 부호가 갈릴 수 있었다. 상세는 docs/M2_COMMAND_SNAPSHOT.md.
//
// 해결: core 0이 한 tick의 결정을 MotorCommandSnapshot 하나로 원자 게시하고,
// core 1이 통째로 복사한 뒤 이 함수만 통과시킨다. 기어는 복사본의 단일 필드라
// 좌우 방향이 갈릴 수 없다.

// core 0이 제어 tick마다 한 번 게시한다. 여기 담긴 값은 모두 같은 순간의 것이다.
struct MotorCommandSnapshot {
    uint32_t seq = 0;             // 게시마다 1 증가. 좌우 동일 tick 확인용 진단
    uint32_t published_ms = 0;    // 게시 시각. 신선도 판정에 쓴다
    float    left_a = 0.0f;
    float    right_a = 0.0f;
    Gear     gear = Gear::Neutral;
    bool     safety_allow = false;              // 같은 tick의 torque_allowed()
    bool     throttle_signal_valid = false;
    bool     propulsion_direction_armed = false;
};
// 주의: 브레이크·컨트롤러 fault·피드백 stale은 여기에 없다. 이미 상위에서
// 명령값 자체를 0으로 만든다(브레이크는 longitudinal, 나머지는 drive_supervisor).
// 여기서 다시 막으면 회생 시 제동 중 명령까지 차단되므로 중복 게이트를 두지 않는다.

// core 1(life 태스크)이 자기 소유로 관리하는 게이트. 공유 상태가 아니다.
struct MotorCommandGates {
    bool  scheduler_alive = false;
    bool  reconnect_inhibit = true;
    bool  component_test_inhibit = true;
    bool  snapshot_fresh = false;
    float reconnect_ramp_scale = 1.0f;   // 램프 중이 아니면 1.0
};

struct MotorCommandParams {
    int drive_target_speed_rpm = 4000;
    int regen_target_speed_rpm = 0;
};

struct MotorFrameCommand {
    float left_a = 0.0f;
    float right_a = 0.0f;
    int   target_rpm_L = 0;
    int   target_rpm_R = 0;
    bool  run_L = false;
    bool  run_R = false;
    bool  normal_allow = false;
};

// 불허 판정이면 좌우 모두 0 A / run=false / 0 rpm으로 나간다. 한쪽만 차단하는
// 경로는 없다. 한쪽만 0으로 만드는 것이 곧 좌우 비대칭이기 때문이다.
MotorFrameCommand motor_command_resolve(const MotorCommandSnapshot &snapshot,
                                        const MotorCommandGates &gates,
                                        const MotorCommandParams &params);
