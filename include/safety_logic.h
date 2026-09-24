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
    bool start_pressed;  // valid throttle released for the required samples
    bool throttle_valid; // false = disconnected/failed throttle signal
    bool previously_driven = false; // never bypass initial release via Halt
};

SafetyState safety_step(SafetyState cur, const SafetyInputs &in);

bool torque_allowed();   // runtime helper, defined in core/safety.cpp
void safety_require_rearm(); // scheduler-only; requires released pedal again
// Bench component tests may run before the aggregate two-controller Drive
// state, but not while the safety FSM is in Halt.
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
    bool     regen_allowed = false; // 검증/Cluster 요청/BMS 유효 + 유효 스로틀 0%
    bool     forward_rotation = false; // 양쪽 fresh, 실차 전진 극성 L>threshold/R<-threshold
    uint16_t block_reasons = 0; // upstream zero-output reasons, same tick
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
    uint16_t block_reasons = 0;
};

enum MotorBlock : uint16_t {
    BLOCK_THROTTLE = 1U << 0, BLOCK_SAFETY = 1U << 1,
    BLOCK_DIRECTION = 1U << 2, BLOCK_FEEDBACK = 1U << 3,
    BLOCK_FAULT = 1U << 4, BLOCK_SPEED_MODE = 1U << 5,
    BLOCK_SNAPSHOT = 1U << 6, BLOCK_HEARTBEAT = 1U << 7,
    BLOCK_RECONNECT = 1U << 8, BLOCK_TEST = 1U << 9,
    BLOCK_NONFINITE = 1U << 10, BLOCK_THERMAL = 1U << 11,
    BLOCK_PADDOCK_SENSOR = 1U << 12
};

// Scheduler-owned stable dwell. A rejected reset is never queued for later.
struct RearmDwell { bool tracking = false; uint32_t since_ms = 0; };
bool fault_rearm_dwell(bool healthy, uint32_t now, RearmDwell &state);

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
