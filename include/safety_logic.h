#pragma once
// [LOCKED] Safety state machine. Pure transition logic is host-tested.

enum class SafetyState { Idle, Ready, Drive, Halt };

struct SafetyInputs {
    bool shutdown_ok;    // shutdown loop closed
    bool handshaked;     // controller handshake done
    bool deadman_ok;     // control commands fresh
    bool start_pressed;  // Start button latched
};

SafetyState safety_step(SafetyState cur, const SafetyInputs &in);

bool torque_allowed();   // runtime helper, defined in core/safety.cpp
