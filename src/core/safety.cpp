#include <Arduino.h>
#include "safety_logic.h"
#include "can_bus.h"

// [LOCKED] Reads hardware safety signals, steps the FSM, gates torque.
namespace {
    constexpr int PIN_SHUTDOWN_SENSE = 34;   // NODE_A feedback (input only)
    constexpr int PIN_START_BTN      = 35;
    SafetyState g_state = SafetyState::Idle;
}

bool torque_allowed() { return g_state == SafetyState::Drive; }

void safety_update() {
    SafetyInputs in{
        digitalRead(PIN_SHUTDOWN_SENSE) == HIGH,
        can_bus::handshaked(),
        can_bus::deadman_ok(),
        digitalRead(PIN_START_BTN) == HIGH,
    };
    g_state = safety_step(g_state, in);
}
