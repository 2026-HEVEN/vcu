#pragma once

// ============================================================
//  REAL-CAR CALIBRATION — 실차에서 바꿀 숫자의 단일 진입점
// ============================================================
//
// 이 파일에는 실차 시험 중 실제로 조정할 가능성이 있는 값만 둔다.
// 고정 제원, CAN/태스크 타이밍, 시험 전용 설정은 fixed_config.h에 있다.
//
// 센서가 림 안쪽에 있어도 휠과 같은 각속도로 돈다. 센서 장착 반경은
// 차속 환산에 사용하지 않는다. WSS는 pulses_per_wheel_rev로 회전수를 만들고,
// 차속은 타이어의 유효 구름반경으로 별도 환산한다.

namespace realcar_cal {

// Temporary bring-up profile for the current dual-motor vehicle.
// Set these back to production requirements as hardware is installed.
namespace bringup {
constexpr bool BRAKE_SENSOR_INSTALLED = true;
// PCB V3 connects the gear selector to GPIO32. Stable Drive
// and Reverse classifications can grant propulsion after the stopped,
// released-throttle direction interlock; Neutral/invalid readings halt it.
constexpr bool GEAR_SELECTOR_INSTALLED = true;
constexpr unsigned GEAR_STABLE_SAMPLES = 10U;  // 100 ms at 100 Hz
constexpr unsigned GEAR_DIRECTION_ARM_SAMPLES = 30U;  // 300 ms at 100 Hz
constexpr int GEAR_DIRECTION_CHANGE_MAX_RPM = 50;
// Forward motor RPM on BOTH sides required for throttle-off regen. Below this
// speed, return to zero-current coasting rather than commanding 0 rpm torque.
constexpr int REGEN_MIN_FORWARD_RPM = 50;
// SUM of both requested phase-current magnitudes, before TV/final correction.
// Level 0 is always OFF. These are NOT battery charge-current limits.
constexpr float REGEN_LEVEL1_TOTAL_CURRENT_A = 300.0f;
constexpr float REGEN_LEVEL2_TOTAL_CURRENT_A = 600.0f;
constexpr float REGEN_LEVEL3_TOTAL_CURRENT_A = 1000.0f;
// Preserve the existing Efficiency-mode regen cap (not Paddock).
constexpr float REGEN_EFF_TOTAL_CURRENT_CAP_A = 30.0f;
// true: use levels 1/2/3; false: any nonzero rotary position uses level 3.
constexpr bool REGEN_FOUR_STAGE_ENABLED = true;
// true: qualified throttle release; false: release AND installed brake ON.
// Positive throttle always cancels regeneration in either mode.
constexpr bool REGEN_ONE_PEDAL_ENABLED = true;
// Total age since last valid cluster command, not an additional grace period.
constexpr unsigned REGEN_REQUEST_STALE_MS = 1000U;
// Provisional mechanical-power ceiling: |I| <= P/(Kt*|omega|).
// Not measured battery charging power; validate Kt/current units and pack limits.
// Zero disables this independent regen ceiling.
constexpr float REGEN_MECHANICAL_POWER_PER_MOTOR_W = 4000.0f;
// Regen only: >= this pack/HV voltage disables negative-current commands.
constexpr float REGEN_VOLTAGE_CUTOFF_V = 57.8f;
// Initial values assume the PCB scales 0/2.5/5 V to approximately
// 0/half/full ESP32 ADC range. They are placeholders until measured.
// Contiguous gear-ladder boundaries. The classifier interprets these as:
// Neutral [0, REVERSE), Reverse [REVERSE, DRIVE), Drive [DRIVE, 4095].
constexpr unsigned GEAR_REVERSE_ADC = 500U;
constexpr unsigned GEAR_DRIVE_ADC = 2500U;
// Keep false until motor polarity and battery charge acceptance are validated.
// Even when true, regen still requires a valid BMS report and low enough SOC.
constexpr bool REGEN_HARDWARE_VALIDATED = true;
// Throttle command ceiling, per motor. This is a software test limit, not a
// competition-rule or battery-current limit. Raise/lower only here after
// checking controller, motor, battery/BMS and energy-meter data.
constexpr float DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A = 500.0f;
constexpr float DRIVE_PHASE_CURRENT_EFF_PER_MOTOR_A = 100.0f;
// Right controller appears to under-read its phase current by about 12%
// (2026-09-29 wheels-up blips: 13-20% more rpm/s per reported A than left,
// and about 25% more motor heat at equal reported current). The controller's
// gear current % is ignored in CAN mode, so the correction is applied here to
// the final right command (drive and regen). Set 1.0 to disable.
constexpr float RIGHT_MOTOR_CURRENT_SCALE = 0.88f;
// Apply the same launch slew limit in Normal and Paddock modes. At the 500 A
// per-motor ceiling this gives 1000 A/s and reaches full demand in 0.5 s.
// Only rising propulsion magnitude is limited; release and protection cuts
// remain immediate.
constexpr float DRIVE_CURRENT_RISE_TIME_S = 1.0f;
// Keep the model-based normal-drive limiter disabled until the official
// Energy Meter path has been driven, time-aligned, and validated. The 8 kW
// value is retained only as the next test calibration; false means no normal
// drive power scaling is applied.
// rpm-scheduled phase-current ceiling (10 kW rule: 500 ms moving-average
// input power). Per-motor DC input power = phase current * (OFFSET +
// PER_RPM * |rpm|) [W/A], fitted on the 2026-10-01 logs in the 200-320 A
// phase-current range (upper 90 % envelope; median offset is 3.127).
// Flux weakening is off, so the model does not depend on pack voltage:
// lower SOC only lowers the reachable rpm, never raises power.
// The controller's own bus-current loop overshot 90 A to 156 A for
// 0.1-0.7 s on every launch, so this caps the command before that.
// 4600 W/motor replays to <= 9.0 kW 500 ms average on both logs.
// Cap per motor: ~330 A at 1200 rpm, ~285 A at 1500 rpm, ~250 A at 1800 rpm.
// 0 disables. Keep the controller Max bus current above ~100 A so its loop
// stays a backstop and does not bind below this cap at low pack voltage.
constexpr float RPM_CAP_POWER_PER_MOTOR_W = 4600.0f;
constexpr float RPM_CAP_W_PER_A_OFFSET = 4.676f;
constexpr float RPM_CAP_W_PER_A_PER_RPM = 7.68e-3f;
constexpr bool ENABLE_DRIVE_POWER_LIMIT = false;
constexpr float DRIVE_POWER_SOFT_LIMIT_W = 8000.0f;
constexpr float DRIVETRAIN_EFFICIENCY = 0.92f;
// EZkontrol target-speed field controls speed; current is the allowed ceiling.
// Pedal maps to 0..5 km/h in either propulsion gear. Verify on raised wheels.
constexpr float PADDOCK_MAX_SPEED_KPH = 5.0f;
constexpr float PADDOCK_CURRENT_MAX_PER_MOTOR_A = 500.0f;
constexpr float PADDOCK_ENTRY_SPEED_MAX_MPS = 3.0f / 3.6f;
// Feedback-based electrical limiting is disabled for this test profile.
// Zero disables these scalers; voltage/current feedback remains logged.
constexpr float PADDOCK_POWER_SOFT_LIMIT_W = 0.0f;
constexpr float PADDOCK_CONTROLLER_BUS_CURRENT_LIMIT_A = 0.0f;
constexpr float PADDOCK_PACK_CURRENT_LIMIT_A = 0.0f;
// Raw values below this floor are treated as a disconnected/failed signal,
// not as a released pedal.
constexpr unsigned THROTTLE_SIGNAL_VALID_MIN_ADC = 200U;
}  // namespace bringup

namespace provisional {
// Initial Hall-throttle calibration. These values are deliberately
// conservative and MUST be replaced with the actual released/full ADC
// readings from the 5 Hz serial diagnostics before a driven test.
constexpr float THROTTLE_RAW_MIN = 400.0f;
constexpr float THROTTLE_RAW_MAX = 2600.0f;

// 초기값: 구름둘레 1.50 m / 2pi. 운전자 탑승·실사용 공기압 상태에서
// 누적 WSS 펄스와 실주행 거리로 다시 식별한다.
constexpr float WHEEL_SPEED_ROLLING_RADIUS_M = 0.2387f;
// 토크->타이어 종력 환산용 유효반경. 우선 같은 값을 쓰되 별도 이름으로
// 유지하여 필요할 때 차속용 반경과 독립 보정할 수 있게 한다.
constexpr float TV_FORCE_RADIUS_M = 0.2387f;

// 효건 탑승 평지 코너웨이트(2026-09-27): FL 68.1, FR 61.0,
// RL 59.7, RR 68.0 kg. CG 높이는 앞축 14도 들기 시험의 잠정 추정치다.
// 정하중 타이어 반경 미측정 및 경사 시험의 총중량 불일치 때문에 재측정 전 확정값이 아니다.
constexpr float VEHICLE_MASS_WITH_DRIVER_KG = 256.8f;
constexpr float WHEELBASE_M = 1.530f;
constexpr float FRONT_TRACK_M = 1.140f;
constexpr float REAR_TRACK_M = 1.090f;
constexpr float CG_HEIGHT_M = 0.410f;  // provisional ground-to-CG height, not IMU mounting height
constexpr float REAR_STATIC_WEIGHT_FRACTION = 127.7f / 256.8f;
constexpr float REAR_LLTD = 0.50f;

// 24 PPR를 10 ms마다 직접 RPM으로 바꾸면 한 펄스 차이가 약 250 rpm이다.
// 아래 1차 필터로 펄스 양자화가 차속/슬립 판정에 그대로 들어가는 것을 막는다.
// 실차에서는 응답 지연과 속도 노이즈를 함께 보고 조정한다.
constexpr float WSS_FILTER_TIME_CONSTANT_S = 0.25f;

// 조향 Unit(+/-1)과 실제 평균 전륜 조향각의 초기 매핑.
// 직진/좌최대/우최대 실측 전에는 TV 게인을 0으로 유지한다.
// 10-01 로그 세 개(반시계 트랙, 원선회, 슬라럼)에서 기구학 모델 대비 실측 요가
// 일관되게 약 0.79배였다(유효 휠베이스 약 2.0 m). 조향계 유격·컴플라이언스·
// 앞바퀴 슬립을 포함한 실효 조향각으로 0.40 rad(약 23도)를 쓴다.
// 기하학적 풀락 바퀴각(좌 약 30도, 우 약 25도)과는 다른 값이다.
constexpr float MAX_ROAD_WHEEL_STEER_RAD = 0.40f;

// 최신 하네스: D25의 12-bit ADC 슬라이드 포텐셔미터. GPIO-fixed의 기존
// SteerRaw 계약에 맞춰 드라이버가 14-bit로 스케일한다. 직진/좌최대/우최대
// raw를 측정한 뒤 center와 counts_per_unit을 교체한다.
// 2026-10-01 로그 두 개로 역산한 센터가 직전 코너 방향에 따라 다르다
// (백래시 약 +/-0.065 unit, raw12 +/-40):
//   04-07 반시계 트랙(좌회전 위주) 직진: raw12 1583
//   04-58 우회전 원선회 뒤 직진:         raw12 1665
// 두 값의 중간으로 둔다. 링키지 유격을 줄이면 다시 잰다.
constexpr unsigned STEERING_CENTER_COUNTS = 6496U;  // straight ahead: raw12 1624 << 2
// raw12 left=176 (~30deg), right=2974 (~25deg), 측정 당시 center는 1700으로 기록됨.
// Driver expands raw12 by 4; scale fits measured road-wheel travel.
constexpr float STEERING_COUNTS_PER_UNIT = 6060.0f;
constexpr bool STEERING_INVERT = true;
}  // namespace provisional

}  // namespace realcar_cal
