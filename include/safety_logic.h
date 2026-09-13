// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include <cstdint>
#include "modules/gear.h"
// [LOCKED] Safety state machine. Pure transition logic is host-tested.

enum class SafetyState { Idle, Ready, Drive, Halt };

struct SafetyInputs {
    bool shutdown_ok;    // shutdown loop closed
    bool handshaked;     // controller handshake done
    bool deadman_ok;     // control commands fresh
    bool start_pressed;  // Start button latched
    bool throttle_valid; // false = disconnected/failed throttle signal
};

SafetyState safety_step(SafetyState cur, const SafetyInputs &in);

bool torque_allowed();   // runtime helper, defined in core/safety.cpp
// Bench component tests may run before the aggregate two-controller Drive
// state, but never after the safety FSM has latched Halt.
bool component_test_safety_allowed();

// --- 좌·우 모터 명령의 최종 관문 ---
// 배경과 설계는 docs/M2_COMMAND_SNAPSHOT.md.
constexpr int DRIVE_TARGET_SPEED_RPM = 4000;   // 실차 확인: 구동 +4000 rpm
constexpr int REGEN_TARGET_SPEED_RPM = 0;      // 회생은 0 rpm

// core 0이 제어 tick마다 한 번 게시한다. 담긴 값은 모두 같은 순간의 것이다.
// 브레이크·컨트롤러 fault·피드백 stale은 상위(longitudinal, drive_supervisor)가
// 이미 명령값을 0으로 만든다. 중복 게이트를 두면 회생 시 제동 중 명령까지 막힌다.
struct MotorCommandSnapshot {
    uint32_t seq = 0;             // 게시마다 1 증가. 0 = 아직 게시 전
    uint32_t published_ms = 0;
    float    left_a = 0.0f;
    float    right_a = 0.0f;
    Gear     gear = Gear::Neutral;
    bool     safety_allow = false;   // 같은 tick의 torque_allowed()
    bool     throttle_signal_valid = false;
    bool     propulsion_direction_armed = false;
};

// CAN life 태스크(core 1) 소유. 코어 경계를 넘지 않으므로 경쟁이 없다.
struct MotorCommandGates {
    bool  scheduler_alive = false;
    bool  reconnect_inhibit = true;
    bool  component_test_inhibit = true;
    bool  snapshot_fresh = false;
    float reconnect_ramp_scale = 1.0f;
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

// 불허면 좌우 모두 0 A / run=false / 0 rpm. 한쪽만 차단하는 경로는 없다.
MotorFrameCommand motor_command_resolve(const MotorCommandSnapshot &snapshot,
                                        const MotorCommandGates &gates);
