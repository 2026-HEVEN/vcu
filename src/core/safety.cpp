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
        // 하네스 v5에는 이 신호용 VCU GPIO가 없다. shutdown chain은 hard-wire,
        // START는 Cluster/LV PCB 왕복으로 처리한다.
        true,
        can_bus::handshaked(),
        can_bus::deadman_ok(),
        true,
    };
    g_state = safety_step(g_state, in);
}
