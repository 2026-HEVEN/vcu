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
// SUM of both motor phase-current magnitudes. TV OFF => 10 A / 15 A each.
// Retained test strength; not increased without pack charge-current validation.
constexpr float REGEN_TOTAL_CURRENT_NORMAL_A = 300.0f;
constexpr float REGEN_TOTAL_CURRENT_EFF_A = 30.0f;
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
// Apply the same launch slew limit in Normal and Paddock modes. At the 500 A
// per-motor ceiling this gives 1000 A/s and reaches full demand in 0.5 s.
// Only rising propulsion magnitude is limited; release and protection cuts
// remain immediate.
constexpr float DRIVE_CURRENT_RISE_TIME_S = 0.0f;
// Keep the model-based normal-drive limiter disabled until the official
// Energy Meter path has been driven, time-aligned, and validated. The 8 kW
// value is retained only as the next test calibration; false means no normal
// drive power scaling is applied.
constexpr bool ENABLE_DRIVE_POWER_LIMIT = false;
constexpr float DRIVE_POWER_SOFT_LIMIT_W = 8000.0f;
constexpr float DRIVETRAIN_EFFICIENCY = 0.92f;
// EZkontrol target-speed field controls speed; current is the allowed ceiling.
// Pedal maps to 0..5 km/h in either propulsion gear. Verify on raised wheels.
constexpr float PADDOCK_MAX_SPEED_KPH = 5.0f;
constexpr float PADDOCK_CURRENT_MAX_PER_MOTOR_A = 100.0f;
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
constexpr float MAX_ROAD_WHEEL_STEER_RAD = 0.52f;

// 최신 하네스: D25의 12-bit ADC 슬라이드 포텐셔미터. GPIO-fixed의 기존
// SteerRaw 계약에 맞춰 드라이버가 14-bit로 스케일한다. 직진/좌최대/우최대
// raw를 측정한 뒤 center와 counts_per_unit을 교체한다.
constexpr unsigned STEERING_CENTER_COUNTS = 6800U;  // straight ahead: raw12 1700 << 2
constexpr float STEERING_COUNTS_PER_UNIT = 4096.0f;
constexpr bool STEERING_INVERT = false;
}  // namespace provisional

}  // namespace realcar_cal
