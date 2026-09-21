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

// 제어 태스크가 tick마다 게시한다. 전류와 회생 판단 조건도 같은 묶음이다.
// 브레이크+양수 스로틀은 구동 허용: 브레이크만으로 전체 출력을 차단하지 않는다.
struct MotorCommandSnapshot {
    uint32_t seq = 0;             // 게시마다 1 증가. 0 = 아직 게시 전
    uint32_t published_ms = 0;
    float    left_a = 0.0f;
    float    right_a = 0.0f;
    Gear     gear = Gear::Neutral;
    bool     safety_allow = false;   // 같은 tick의 torque_allowed()
    bool     throttle_signal_valid = false;
    bool     propulsion_direction_armed = false;
    bool     brake_active = false;
    bool     regen_allowed = false; // 설치/검증/요청/BMS 유효 + 유효 스로틀 0%
    bool     forward_rotation = false; // 양쪽 fresh 피드백, 두 RPM 모두 양수
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

// 명령을 복사한 다음 읽은 시각을 전달한다. 미게시 또는 유효기간 초과면 false.
bool motor_snapshot_fresh(const MotorCommandSnapshot &snapshot,
                          uint32_t checked_at_ms, uint32_t max_age_ms);
enum class MotorTxResult : uint8_t { Skipped, Queued, Failed };
struct MotorTxSideDiagnostics {
    MotorTxResult result = MotorTxResult::Skipped;
    uint32_t failed_total = 0;
    unsigned consecutive_failures = 0;
    bool queued_valid = false;
    float last_queued_a = 0;
    int last_queued_rpm = 0;
    bool last_queued_running = false;
    uint32_t last_queued_seq = 0;
    uint32_t last_queued_ms = 0;
};
struct MotorTxDiagnostics {
    MotorFrameCommand requested{};
    uint32_t seq = 0;
    uint32_t snapshot_age_ms = 0;
    bool snapshot_fresh = false;
    uint32_t stale_total = 0;
    MotorTxSideDiagnostics left{}, right{};
};
void motor_tx_record(MotorTxSideDiagnostics &out, MotorTxResult result,
                     float amps, int rpm, bool running, uint32_t seq, uint32_t now);
