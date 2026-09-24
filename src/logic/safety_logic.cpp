// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "safety_logic.h"
#include "modules/motor_direction.h"
#include <cmath>

bool motor_snapshot_fresh(const MotorCommandSnapshot &snapshot,
                          uint32_t checked_at_ms, uint32_t max_age_ms) {
    // seq=0은 아직 제어 루프가 명령을 게시하지 않은 초기 상태다.
    if (snapshot.seq == 0U) return false;

    // 명령 나이 = 검사 시각 - 명령 게시 시각.
    // uint32_t 뺄셈은 millis()가 최댓값에서 0으로 돌아가는 경우도 처리한다.
    const uint32_t command_age_ms = checked_at_ms - snapshot.published_ms;
    return command_age_ms <= max_age_ms;
}

void motor_tx_record(MotorTxSideDiagnostics &out, MotorTxResult result,
                     float amps, int rpm, bool running, uint32_t seq, uint32_t now) {
    out.result = result;
    if (result == MotorTxResult::Failed) {
        if (out.failed_total != UINT32_MAX) ++out.failed_total;
        if (out.consecutive_failures != UINT32_MAX) ++out.consecutive_failures;
    } else {
        out.consecutive_failures = 0;
    }
    if (result == MotorTxResult::Queued) {
        out.queued_valid = true;
        out.last_queued_a = amps;
        out.last_queued_rpm = rpm;
        out.last_queued_running = running;
        out.last_queued_seq = seq;
        out.last_queued_ms = now;
    }
}

SafetyState safety_step(SafetyState cur, const SafetyInputs &in) {
    // A lost throttle signal stops torque immediately. Unlike initial arming,
    // recovery from Halt may resume with the pedal held once the signal,
    // controller handshakes and command heartbeat have all recovered.
    if (!in.shutdown_ok || !in.throttle_valid) return SafetyState::Halt;
    switch (cur) {
        case SafetyState::Idle:
            return in.handshaked ? SafetyState::Ready : SafetyState::Idle;
        case SafetyState::Ready:
            return in.start_pressed && in.handshaked && in.deadman_ok
                ? SafetyState::Drive : SafetyState::Ready;
        case SafetyState::Drive:
            return in.deadman_ok ? SafetyState::Drive : SafetyState::Halt;
        case SafetyState::Halt:
            return in.handshaked && in.deadman_ok &&
                (in.previously_driven || in.start_pressed)
                ? SafetyState::Drive : SafetyState::Halt;
        default:
            return SafetyState::Halt;
    }
}

bool fault_rearm_dwell(bool healthy, uint32_t now, RearmDwell &s) {
    if (!healthy) { s.tracking = false; return false; }
    if (!s.tracking) { s.tracking = true; s.since_ms = now; }
    return now - s.since_ms >= 1000U;
}

namespace {
float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

}  // namespace

MotorFrameCommand motor_command_resolve(const MotorCommandSnapshot &snapshot,
                                        const MotorCommandGates &gates) {
    MotorFrameCommand out;
    // Clamped<> 는 NaN을 막지 못한다(NaN 비교가 전부 거짓이라 통과). 마지막 검사.
    const bool finite_command =
        std::isfinite(snapshot.left_a) && std::isfinite(snapshot.right_a);
    const bool propulsion_gear =
        snapshot.gear == Gear::Drive || snapshot.gear == Gear::Reverse;

    out.block_reasons = snapshot.block_reasons;
    if (!snapshot.safety_allow) out.block_reasons |= BLOCK_SAFETY;
    if (!snapshot.throttle_signal_valid) out.block_reasons |= BLOCK_THROTTLE;
    if (!snapshot.propulsion_direction_armed || !propulsion_gear)
        out.block_reasons |= BLOCK_DIRECTION;
    if (!gates.scheduler_alive) out.block_reasons |= BLOCK_HEARTBEAT;
    if (!gates.snapshot_fresh) out.block_reasons |= BLOCK_SNAPSHOT;
    if (gates.reconnect_inhibit) out.block_reasons |= BLOCK_RECONNECT;
    if (gates.component_test_inhibit) out.block_reasons |= BLOCK_TEST;
    if (!finite_command) out.block_reasons |= BLOCK_NONFINITE;
    out.normal_allow = out.block_reasons == 0;
    if (!out.normal_allow) return out;   // 기본값이 곧 좌우 동시 차단

    const float ramp = std::isfinite(gates.reconnect_ramp_scale)
        ? clamp01(gates.reconnect_ramp_scale) : 0.0f;
    // dev의 D 회생/R 구동 부호 규칙을 재사용한다. 공유 state는 다시 읽지 않는다.
    const auto left = motor_direction_command(snapshot.left_a * ramp,
        snapshot.gear, true, snapshot.regen_allowed, snapshot.forward_rotation);
    const auto right = motor_direction_command(snapshot.right_a * ramp,
        snapshot.gear, true, snapshot.regen_allowed, snapshot.forward_rotation);
    out.left_a = left.current_a;
    out.right_a = right.current_a;
    out.run_L = left.running;
    out.run_R = right.running;
    out.target_rpm_L = left.target_rpm;
    out.target_rpm_R = right.target_rpm;
    return out;
}
