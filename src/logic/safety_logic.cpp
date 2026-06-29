// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "safety_logic.h"

SafetyState safety_step(SafetyState cur, const SafetyInputs &in) {
    // Any loss of the shutdown loop or deadman -> immediate Halt (hard rule).
    if (!in.shutdown_ok) return SafetyState::Halt;
    switch (cur) {
        case SafetyState::Idle:
            return in.handshaked ? SafetyState::Ready : SafetyState::Idle;
        case SafetyState::Ready:
            return in.start_pressed ? SafetyState::Drive : SafetyState::Ready;
        case SafetyState::Drive:
            return in.deadman_ok ? SafetyState::Drive : SafetyState::Halt;
        case SafetyState::Halt:
        default:
            return SafetyState::Halt;   // Halt is latched; re-arm via power cycle / Start
    }
}
