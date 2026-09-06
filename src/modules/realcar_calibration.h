#pragma once

// ============================================================
//  REAL-CAR CALIBRATION — 실차에서 바꿀 숫자의 단일 진입점
// ============================================================
//
// 이 파일의 값은 두 그룹으로 나뉜다.
//   confirmed   : 부품/장착 사양으로 확정. 하드웨어가 바뀔 때만 수정.
//   provisional : CarMaker/BOM/초기 측정값. 실차 식별 결과로 갱신.
//
// 센서가 림 안쪽에 있어도 휠과 같은 각속도로 돈다. 센서 장착 반경은
// 차속 환산에 사용하지 않는다. WSS는 pulses_per_wheel_rev로 회전수를 만들고,
// 차속은 타이어의 유효 구름반경으로 별도 환산한다.

namespace realcar_cal {

// Temporary bring-up profile for the current dual-motor vehicle.
// Set these back to production requirements as hardware is installed.
namespace bringup {
constexpr bool BRAKE_SENSOR_INSTALLED = false;
// The completed harness connects the gear selector to GPIO27. Stable Drive
// and Reverse classifications can grant propulsion after the stopped,
// released-throttle direction interlock; Neutral/invalid readings halt it.
constexpr bool GEAR_SELECTOR_INSTALLED = true;
constexpr unsigned GEAR_STABLE_SAMPLES = 10U;  // 100 ms at 100 Hz
constexpr unsigned GEAR_DIRECTION_ARM_SAMPLES = 30U;  // 300 ms at 100 Hz
constexpr int GEAR_DIRECTION_CHANGE_MAX_RPM = 50;
// Initial values assume the PCB scales 0/2.5/5 V to approximately
// 0/half/full ESP32 ADC range. They are placeholders until measured.
// Contiguous gear-ladder boundaries. The classifier interprets these as:
// Neutral [0, REVERSE), Reverse [REVERSE, DRIVE), Drive [DRIVE, 4095].
constexpr unsigned GEAR_NEUTRAL_ADC = 0U;
constexpr unsigned GEAR_REVERSE_ADC = 500U;
constexpr unsigned GEAR_DRIVE_ADC = 2500U;
constexpr unsigned GEAR_ADC_TOLERANCE = 0U;  // reserved; no dead band
// Brake/BMS/current polarity and charge limits are not validated yet. Keeping
// this false makes a Cluster Regen-Auto request observable but unable to
// produce negative phase current.
constexpr bool REGEN_HARDWARE_VALIDATED = false;
// Do not arm or send propulsion until both independently addressed
// EZkontrol units have completed their 0x55/0xAA handshake.
constexpr bool REQUIRE_BOTH_MOTOR_CONTROLLERS = true;
// Throttle command ceiling, per motor. This is a software test limit, not a
// competition-rule or battery-current limit. Raise/lower only here after
// checking controller, motor, battery/BMS and energy-meter data.
constexpr float DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A = 500.0f;
constexpr float DRIVE_PHASE_CURRENT_EFF_PER_MOTOR_A = 100.0f;
// Bench-only serial motor pulse used by bringup/component-test. A pulse is
// accepted only with released throttle, fresh CAN feedback, no controller
// fault and a nearly stopped selected motor. The CAN life task re-checks the
// runtime gates at MOTOR_COMMAND_PERIOD_MS and always expires at the deadline.
constexpr float COMPONENT_TEST_CURRENT_MAX_PER_MOTOR_A = 150.0f;
constexpr unsigned COMPONENT_TEST_DURATION_MIN_MS = 100U;
constexpr unsigned COMPONENT_TEST_DURATION_MAX_MS = 3000U;
constexpr int COMPONENT_TEST_START_MAX_MOTOR_RPM = 50;
// Keep the estimated power limiter OFF during bring-up so logged controller
// feedback and the official Energy Meter can be compared without the estimator
// changing the test. Set true only after the power model is validated.
constexpr bool ENABLE_DRIVE_POWER_LIMIT = false;
// Candidate limit used when ENABLE_DRIVE_POWER_LIMIT is true. The official
// Energy Meter remains authoritative.
constexpr float DRIVE_POWER_SOFT_LIMIT_W = 9000.0f;
constexpr float DRIVETRAIN_EFFICIENCY = 0.92f;
constexpr float CONTROLLER_FEEDBACK_STALE_MS = 250.0f;
// Motor-controller command/life frame cadence. Set 10 ms for 100 Hz or 50 ms
// for the vendor protocol's original nominal 20 Hz cadence.
constexpr unsigned MOTOR_COMMAND_PERIOD_MS = 50U; //모터 캔 제어주기
static_assert(MOTOR_COMMAND_PERIOD_MS > 0U,
              "motor command period must be nonzero");
// If either feedback part remains absent this long after a handshake, stop
// that controller's normal command traffic so it can return to its 0x55
// handshake state. This exceeds both the normal freshness gate and the
// controller's documented 250--500 ms command/life timeout.
constexpr unsigned CONTROLLER_REHANDSHAKE_TIMEOUT_MS = 750U;
// After a component test, require this much released-pedal time before normal
// throttle control can resume. The tick count follows the command period.
constexpr unsigned COMPONENT_TEST_RELEASE_HOLD_MS = 300U;
constexpr unsigned COMPONENT_TEST_RELEASE_TICKS =
    (COMPONENT_TEST_RELEASE_HOLD_MS + MOTOR_COMMAND_PERIOD_MS - 1U) /
    MOTOR_COMMAND_PERIOD_MS;
// A controller reconnect no longer requires pedal release. Once both protocol
// handshakes and feedback streams are healthy, restore the held driver demand
// gradually from zero over this interval instead of applying a torque step.
constexpr unsigned MOTOR_RECONNECT_RAMP_MS = 1000U;
static_assert(MOTOR_RECONNECT_RAMP_MS > 0U,
              "motor reconnect ramp must be nonzero");
constexpr float CLUSTER_COMMAND_STALE_MS = 200.0f;
constexpr unsigned CAN_RX_QUEUE_LENGTH = 32U;
constexpr float PHASE_CURRENT_HARD_CUTOFF_A = 1000.0f;
// Energy Meter (100 Hz) and Monolith/controller feedback (20 Hz) time-axis
// marker. It never runs automatically: Serial SYNC_ARM followed by SYNC_RUN
// is required, and the runtime safety conditions are checked every 10 ms.
// The initial 20 A per motor is provisional; calibrate on stands/rollers so
// the HV bus-current pulse is visible without an unsafe wheel acceleration.
constexpr bool ENABLE_TIME_SYNC_PULSE = true;
constexpr float TIME_SYNC_PHASE_CURRENT_PER_MOTOR_A = 20.0f;
constexpr float TIME_SYNC_PULSE_ON_S = 0.5f;
constexpr float TIME_SYNC_PULSE_OFF_S = 0.5f;
constexpr unsigned TIME_SYNC_PULSE_COUNT = 3U;
constexpr float TIME_SYNC_ARM_TIMEOUT_S = 10.0f;
constexpr float TIME_SYNC_START_SPEED_MAX_MPS = 1.0f / 3.6f;
constexpr float CONTROLLER_DERATE_START_C = 75.0f;
constexpr float CONTROLLER_CUTOFF_C = 85.0f;
constexpr float MOTOR_DERATE_START_C = 100.0f;
constexpr float MOTOR_CUTOFF_C = 120.0f;
// Provisional speed/current test envelope.  The phase-current ceiling falls
// continuously from 500 A/motor at standstill to 50 A/motor at 80 km/h, then
// holds 50 A/motor above that speed.  This is only active in paddock mode.
constexpr bool PADDOCK_CURRENT_CALIBRATED = false;
constexpr float PADDOCK_CURRENT_ZERO_SPEED_PER_MOTOR_A = 500.0f;
constexpr float PADDOCK_CURRENT_HIGH_SPEED_PER_MOTOR_A = 50.0f;
constexpr float PADDOCK_CURRENT_LINEAR_END_SPEED_MPS = 80.0f / 3.6f;
// 500 A / 0.5 s = 1000 A/s. Only rising propulsion magnitude is limited;
// release, faults and protective reductions remain immediate.
constexpr float PADDOCK_CURRENT_RISE_TIME_S = 0.5f;
constexpr float PADDOCK_ENTRY_SPEED_MAX_MPS = 3.0f / 3.6f;
// The Bexel pack's 157 A continuous rating is about 8.13 kW at 51.8 V.
// Keep a small margin below that value; this is a soft command scaler, not a
// substitute for hardware over-current protection.
constexpr float PADDOCK_POWER_SOFT_LIMIT_W = 8000.0f;
// Controller L+R bus-current feedback ran about 20--30% above the delayed BMS
// value in the 2026-09-05 logs.  200 A controller-sum and 150 A BMS limits
// represent approximately the same operating boundary.
constexpr float PADDOCK_CONTROLLER_BUS_CURRENT_LIMIT_A = 200.0f;
constexpr float PADDOCK_PACK_CURRENT_LIMIT_A = 150.0f;
constexpr bool PADDOCK_REQUIRE_PACK_DATA = true;
// EZkontrol uses -40 C as the missing/invalid temperature sentinel.
constexpr float TELEMETRY_TEMPERATURE_VALID_MIN_C = -30.0f;
// Vehicle cannot enter Drive until the released throttle has stayed below
// this threshold for THROTTLE_ARM_CONSECUTIVE_TICKS scheduler passes.
constexpr float THROTTLE_ARM_MAX_PCT = 1.0f;
constexpr unsigned THROTTLE_ARM_CONSECUTIVE_TICKS = 30U;  // about 300 ms at 100 Hz
// Raw values below this floor are treated as a disconnected/failed signal,
// not as a released pedal. Values from 400 through the 0% point at 500 are
// accepted as a valid released pedal while still commanding zero current.
constexpr unsigned THROTTLE_SIGNAL_VALID_MIN_ADC = 400U;
}  // namespace bringup

namespace confirmed {
// PCNT는 상승엣지만 센다. 48개의 N/S 교차 극에서 실측되는 상승엣지는
// 네 바퀴 모두 휠 1회전당 24개다.
constexpr float WSS_PULSES_PER_WHEEL_REV_FL = 24.0f;
constexpr float WSS_PULSES_PER_WHEEL_REV_FR = 24.0f;
constexpr float WSS_PULSES_PER_WHEEL_REV_RL = 24.0f;
constexpr float WSS_PULSES_PER_WHEEL_REV_RR = 24.0f;

constexpr float GEAR_RATIO = 3.72f;
constexpr float MOTOR_KT_NM_PER_A = 0.1266f;
constexpr float MOTOR_CONTINUOUS_CURRENT_MAX_A = 103.0f;
constexpr float CONTROL_PERIOD_S = 0.01f;  // 100 Hz
}  // namespace confirmed

namespace provisional {
// Initial Hall-throttle calibration. These values are deliberately
// conservative and MUST be replaced with the actual released/full ADC
// readings from the 5 Hz serial diagnostics before a driven test.
constexpr float THROTTLE_RAW_MIN = 500.0f;
constexpr float THROTTLE_RAW_MAX = 3000.0f;

// 초기값: 구름둘레 1.50 m / 2pi. 운전자 탑승·실사용 공기압 상태에서
// 누적 WSS 펄스와 실주행 거리로 다시 식별한다.
constexpr float WHEEL_SPEED_ROLLING_RADIUS_M = 0.2387f;
// 토크->타이어 종력 환산용 유효반경. 우선 같은 값을 쓰되 별도 이름으로
// 유지하여 필요할 때 차속용 반경과 독립 보정할 수 있게 한다.
constexpr float TV_FORCE_RADIUS_M = 0.2387f;

// CarMaker BOM197/설계 형상에서 가져온 실차 시험 시작값. 줄자·코너웨이트
// 및 CG 식별 결과가 나오면 이 파일만 수정한다.
constexpr float VEHICLE_MASS_WITH_DRIVER_KG = 197.345f;
constexpr float WHEELBASE_M = 1.530f;
constexpr float FRONT_TRACK_M = 1.140f;
constexpr float REAR_TRACK_M = 1.090f;
constexpr float CG_HEIGHT_M = 0.2800873f;
constexpr float REAR_STATIC_WEIGHT_FRACTION = 0.6180974f;
constexpr float REAR_LLTD = 0.50f;

// 차속 추정의 샘플 간 물리 타당성 검사. 실차 로그의 최대 종가속도와
// WSS 양자화가 확보된 뒤 조정한다.
constexpr float VEHICLE_SPEED_MAX_ACCEL_MPS2 = 15.0f;

// 24 PPR를 10 ms마다 직접 RPM으로 바꾸면 한 펄스 차이가 약 250 rpm이다.
// 아래 1차 필터로 펄스 양자화가 차속/슬립 판정에 그대로 들어가는 것을 막는다.
// 실차에서는 응답 지연과 속도 노이즈를 함께 보고 조정한다.
constexpr float WSS_FILTER_TIME_CONSTANT_S = 0.25f;

// 조향 Unit(+/-1)과 실제 평균 전륜 조향각의 초기 매핑.
// 직진/좌최대/우최대 실측 전에는 TV 게인을 0으로 유지한다.
constexpr float MAX_ROAD_WHEEL_STEER_RAD = 0.52f;

// 최신 하네스: D25의 12-bit ADC 슬라이드 포텐셔미터. GPIO-fixed의 기존
// SteerRaw 계약에 맞춰 드라이버가 14-bit로 스케일한다. 직진/좌최대/우최대
// raw를 측정한 뒤 center와 counts_per_unit을 교체한다.
constexpr unsigned STEERING_CENTER_COUNTS = 8192U;
constexpr float STEERING_COUNTS_PER_UNIT = 4096.0f;
constexpr bool STEERING_INVERT = false;
}  // namespace provisional

}  // namespace realcar_cal
