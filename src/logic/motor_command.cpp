// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "motor_command.h"
#include <cmath>

// [LOCKED] 좌·우 명령 확정. 전역 state와 하드웨어를 모르는 순수 함수라
// test/test_motor_command 가 노트북에서 그대로 돌린다.

namespace {

float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

// 목표 rpm은 측별로 계산한다. 회생(음전류) 여부가 좌우 다를 수 있기 때문이다.
// 다만 gear는 스냅샷의 단일 값이므로 좌우 방향이 갈릴 수 없다.
int target_rpm_for(Gear gear, float amps, const MotorCommandParams &params) {
    if (gear == Gear::Reverse) return -params.drive_target_speed_rpm;
    return amps < 0.0f ? params.regen_target_speed_rpm
                       : params.drive_target_speed_rpm;
}

}  // namespace

MotorFrameCommand motor_command_resolve(const MotorCommandSnapshot &snapshot,
                                        const MotorCommandGates &gates,
                                        const MotorCommandParams &params) {
    MotorFrameCommand out;

    // NaN/Inf는 Clamped<> 도메인 타입이 막지 못한다. NaN과의 비교는 모두 거짓이라
    // 범위 클램프를 그대로 통과한다. 명령이 프레임에 실리기 전 마지막 검사다.
    const bool finite_command =
        std::isfinite(snapshot.left_a) && std::isfinite(snapshot.right_a);
    const bool propulsion_gear =
        snapshot.gear == Gear::Drive || snapshot.gear == Gear::Reverse;

    out.normal_allow =
        snapshot.safety_allow &&
        snapshot.throttle_signal_valid &&
        snapshot.propulsion_direction_armed &&
        propulsion_gear &&
        gates.scheduler_alive &&
        gates.snapshot_fresh &&
        !gates.reconnect_inhibit &&
        !gates.component_test_inhibit &&
        finite_command;

    // 불허면 기본값 그대로 반환한다. 좌우 모두 0 A / run=false / 0 rpm.
    if (!out.normal_allow) return out;

    const float ramp_scale = std::isfinite(gates.reconnect_ramp_scale)
        ? clamp01(gates.reconnect_ramp_scale) : 0.0f;

    out.left_a  = snapshot.left_a  * ramp_scale;
    out.right_a = snapshot.right_a * ramp_scale;
    out.run_L = true;
    out.run_R = true;
    out.target_rpm_L = target_rpm_for(snapshot.gear, out.left_a, params);
    out.target_rpm_R = target_rpm_for(snapshot.gear, out.right_a, params);
    return out;
}
