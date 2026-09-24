#pragma once

// Stable vehicle facts and firmware policy.
//
// These values are intentionally kept out of realcar_calibration.h: they are
// not track-side tuning knobs. Change them only when the hardware, CAN
// contract, scheduler architecture, or bench-test procedure itself changes.
namespace fixed_config {

namespace vehicle {
// PCNT counts rising edges. The installed alternating-pole rings produce 24
// rising edges per wheel revolution on every channel.
constexpr float WSS_PULSES_PER_WHEEL_REV_FL = 24.0f;
constexpr float WSS_PULSES_PER_WHEEL_REV_FR = 24.0f;
constexpr float WSS_PULSES_PER_WHEEL_REV_RL = 24.0f;
constexpr float WSS_PULSES_PER_WHEEL_REV_RR = 24.0f;

constexpr float GEAR_RATIO = 3.72f;
constexpr float MOTOR_KT_NM_PER_A = 0.1266f;
constexpr float MOTOR_CONTINUOUS_CURRENT_MAX_A = 103.0f;
constexpr float CONTROL_PERIOD_S = 0.01f;  // 100 Hz
}  // namespace vehicle

namespace runtime {
// A dual-motor vehicle never grants propulsion with only one controller ready.
constexpr bool REQUIRE_BOTH_MOTOR_CONTROLLERS = true;
constexpr float CONTROLLER_FEEDBACK_STALE_MS = 250.0f;

// EZkontrol command/life frame cadence. This is a protocol timing constant,
// not a current or vehicle calibration.
constexpr unsigned MOTOR_COMMAND_PERIOD_MS = 50U;
static_assert(MOTOR_COMMAND_PERIOD_MS > 0U,
              "motor command period must be nonzero");

// Link is dropped when BOTH feedback parts are silent this long.
constexpr unsigned CONTROLLER_REHANDSHAKE_TIMEOUT_MS = 750U;
// Each part alone may pause (FB1 ~1 s, FB2 up to ~6 s under load, 2026-09-21
// and 2026-09-24 captures) while the other keeps arriving. These are the
// per-part hard limits; beyond them the link is dropped as well.
constexpr unsigned CONTROLLER_FB1_MAX_AGE_MS = 1500U;
constexpr unsigned CONTROLLER_FB2_MAX_AGE_MS = 7000U;
static_assert(CONTROLLER_REHANDSHAKE_TIMEOUT_MS < CONTROLLER_FB1_MAX_AGE_MS,
              "both-silent limit must be the tighter one");

// Command phase guard (docs/CAN_PHASE_GUARD.md). EZkontrol drops feedback
// when a command arrives ~1-2.5 ms before its FB1 slot. Phases here are
// measured through the 5 ms RX poll, so they read 0-5 ms late: a measured
// phase inside [SAFE_MIN, SAFE_MAX] is at least ~8 ms from the window.
constexpr int CMD_PHASE_SAFE_MIN_MS = 12;
constexpr int CMD_PHASE_SAFE_MAX_MS = 46;
constexpr int CMD_PHASE_TARGET_MIN_MS = 18;
constexpr int CMD_PHASE_TARGET_MAX_MS = 40;
constexpr unsigned CMD_PHASE_CONFIRM_TICKS = 3U;
static_assert(CMD_PHASE_SAFE_MIN_MS <= CMD_PHASE_TARGET_MIN_MS &&
              CMD_PHASE_TARGET_MAX_MS <= CMD_PHASE_SAFE_MAX_MS &&
              CMD_PHASE_SAFE_MAX_MS < (int)MOTOR_COMMAND_PERIOD_MS,
              "phase bands must nest inside one command period");
// Any two phases are at most period/2 apart on the circle, so the safe band
// must be at least that wide for a common shift to always exist.
static_assert(CMD_PHASE_SAFE_MAX_MS - CMD_PHASE_SAFE_MIN_MS >=
              (int)MOTOR_COMMAND_PERIOD_MS / 2,
              "safe band too narrow to fit both controllers");
constexpr unsigned MOTOR_RECONNECT_RAMP_MS = 1000U;
static_assert(MOTOR_RECONNECT_RAMP_MS > 0U,
              "motor reconnect ramp must be nonzero");

constexpr unsigned MOTOR_COMMAND_SNAPSHOT_MAX_AGE_MS = 100U;
static_assert(MOTOR_COMMAND_SNAPSHOT_MAX_AGE_MS > 0U,
              "command snapshot age limit must be nonzero");

constexpr unsigned MOTOR_TX_FAIL_LIMIT = 3U;
static_assert(MOTOR_TX_FAIL_LIMIT * MOTOR_COMMAND_PERIOD_MS < 250U,
              "VCU must cut before the controller life timeout");

constexpr float CLUSTER_COMMAND_STALE_MS = 200.0f;
constexpr unsigned CAN_RX_QUEUE_LENGTH = 32U;

// This is retained as a malformed-feedback/anomaly latch, not as the primary
// motor-current protection. The controller enforces its configured current.
constexpr float PHASE_CURRENT_HARD_CUTOFF_A = 1000.0f;

// Released-pedal arming policy.
constexpr float THROTTLE_ARM_MAX_PCT = 1.0f;
constexpr unsigned THROTTLE_ARM_CONSECUTIVE_TICKS = 30U;

// EZkontrol reports -40 C for absent/invalid temperature telemetry.
constexpr float TELEMETRY_TEMPERATURE_VALID_MIN_C = -30.0f;
}  // namespace runtime

namespace bench {
constexpr float COMPONENT_TEST_CURRENT_MAX_PER_MOTOR_A = 150.0f;
constexpr unsigned COMPONENT_TEST_DURATION_MIN_MS = 100U;
constexpr unsigned COMPONENT_TEST_DURATION_MAX_MS = 3000U;
constexpr int COMPONENT_TEST_START_MAX_MOTOR_RPM = 50;
constexpr unsigned COMPONENT_TEST_RELEASE_HOLD_MS = 300U;
constexpr unsigned COMPONENT_TEST_RELEASE_TICKS =
    (COMPONENT_TEST_RELEASE_HOLD_MS + runtime::MOTOR_COMMAND_PERIOD_MS - 1U) /
    runtime::MOTOR_COMMAND_PERIOD_MS;

// Energy-meter/Monolith time-axis marker. It is serial-command armed and is
// never started automatically.
constexpr bool ENABLE_TIME_SYNC_PULSE = true;
constexpr float TIME_SYNC_PHASE_CURRENT_PER_MOTOR_A = 20.0f;
constexpr float TIME_SYNC_PULSE_ON_S = 0.5f;
constexpr float TIME_SYNC_PULSE_OFF_S = 0.5f;
constexpr unsigned TIME_SYNC_PULSE_COUNT = 3U;
constexpr float TIME_SYNC_ARM_TIMEOUT_S = 10.0f;
constexpr float TIME_SYNC_START_SPEED_MAX_MPS = 1.0f / 3.6f;
}  // namespace bench

}  // namespace fixed_config
