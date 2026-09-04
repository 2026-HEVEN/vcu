// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
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
