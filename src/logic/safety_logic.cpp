// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "safety_logic.h"
#include <cmath>

SafetyState safety_step(SafetyState cur, const SafetyInputs &in) {
    // Any loss of the shutdown loop or deadman -> immediate Halt (hard rule).
    if (!in.shutdown_ok || !in.throttle_valid) return SafetyState::Halt;
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

namespace {
float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

// 회생 여부가 좌우 다를 수 있어 측별로 계산한다. gear는 단일 값이므로 방향은 같다.
int target_rpm_for(Gear gear, float amps) {
    if (gear == Gear::Reverse) return -DRIVE_TARGET_SPEED_RPM;
    return amps < 0.0f ? REGEN_TARGET_SPEED_RPM : DRIVE_TARGET_SPEED_RPM;
}
}  // namespace

MotorFrameCommand motor_command_resolve(const MotorCommandSnapshot &snapshot,
                                        const MotorCommandGates &gates) {
    MotorFrameCommand out;
    // Clamped<> 는 NaN을 막지 못한다(NaN 비교가 전부 거짓이라 통과). 마지막 검사.
    const bool finite_command =
        std::isfinite(snapshot.left_a) && std::isfinite(snapshot.right_a);
    const bool propulsion_gear =
        snapshot.gear == Gear::Drive || snapshot.gear == Gear::Reverse;

    out.normal_allow =
        snapshot.safety_allow && snapshot.throttle_signal_valid &&
        snapshot.propulsion_direction_armed && propulsion_gear &&
        gates.scheduler_alive && gates.snapshot_fresh &&
        !gates.reconnect_inhibit && !gates.component_test_inhibit &&
        finite_command;
    if (!out.normal_allow) return out;   // 기본값이 곧 좌우 동시 차단

    const float ramp = std::isfinite(gates.reconnect_ramp_scale)
        ? clamp01(gates.reconnect_ramp_scale) : 0.0f;
    out.left_a  = snapshot.left_a  * ramp;
    out.right_a = snapshot.right_a * ramp;
    out.run_L = out.run_R = true;
    out.target_rpm_L = target_rpm_for(snapshot.gear, out.left_a);
    out.target_rpm_R = target_rpm_for(snapshot.gear, out.right_a);
    return out;
}
