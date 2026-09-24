// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "safety_logic.h"
#include "can_bus.h"
#include "state.h"
#include "modules/realcar_calibration.h"
#include "modules/fixed_config.h"

// [LOCKED] Reads hardware safety signals, steps the FSM, gates torque.
namespace {
    SafetyState g_state = SafetyState::Idle;
    unsigned g_throttle_release_ticks = 0;
    bool g_previously_driven = false;
}

void safety_require_rearm() {
    g_state = SafetyState::Ready;
    g_throttle_release_ticks = 0;
    g_previously_driven = false;
}

bool torque_allowed() { return g_state == SafetyState::Drive; }
bool component_test_safety_allowed() { return g_state != SafetyState::Halt; }

void safety_update() {
    if (state.throttle_signal_valid &&
        (float)state.throttle_pct <= fixed_config::runtime::THROTTLE_ARM_MAX_PCT) {
        if (g_throttle_release_ticks < fixed_config::runtime::THROTTLE_ARM_CONSECUTIVE_TICKS) {
            ++g_throttle_release_ticks;
        }
    } else {
        g_throttle_release_ticks = 0;
    }

    const bool throttle_released_long_enough =
        g_throttle_release_ticks >= fixed_config::runtime::THROTTLE_ARM_CONSECUTIVE_TICKS;
    SafetyInputs in{
        // 하네스 v5에는 이 신호용 VCU GPIO가 없다. shutdown chain은 hard-wire,
        // START는 Cluster/LV PCB 왕복으로 처리한다.
        true,
        can_bus::handshaked(),
        can_bus::deadman_ok(),
        throttle_released_long_enough,
        state.throttle_signal_valid,
        g_previously_driven,
    };
    g_state = safety_step(g_state, in);
    if (g_state == SafetyState::Drive) g_previously_driven = true;
}
