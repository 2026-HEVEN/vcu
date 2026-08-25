// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "safety_logic.h"
#include "can_bus.h"

// [LOCKED] Reads hardware safety signals, steps the FSM, gates torque.
namespace {
    SafetyState g_state = SafetyState::Idle;
}

bool torque_allowed() { return g_state == SafetyState::Drive; }

void safety_update() {
    SafetyInputs in{
        // Harness v5 has no VCU GPIO for these signals. The shutdown chain is
        // hard-wired and START is handled by the Cluster/LV PCB round trip.
        true,
        can_bus::handshaked(),
        can_bus::deadman_ok(),
        true,
    };
    g_state = safety_step(g_state, in);
}
