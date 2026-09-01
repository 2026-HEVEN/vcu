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
// Current vehicle bring-up is forward-only. Set true only after GPIO27 has
// been measured at N/R/D on the completed PCB and the bands below updated.
constexpr bool GEAR_SELECTOR_INSTALLED = false;
constexpr unsigned GEAR_STABLE_SAMPLES = 10U;  // 100 ms at 100 Hz
// Initial values assume the PCB scales 0/2.5/5 V to approximately
// 0/half/full ESP32 ADC range. They are placeholders until measured.
constexpr unsigned GEAR_NEUTRAL_ADC = 0U;
constexpr unsigned GEAR_REVERSE_ADC = 2048U;
constexpr unsigned GEAR_DRIVE_ADC = 4095U;
constexpr unsigned GEAR_ADC_TOLERANCE = 450U;
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
constexpr float DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A = 300.0f;
constexpr float DRIVE_PHASE_CURRENT_EFF_PER_MOTOR_A = 200.0f;
// The official Energy Meter remains authoritative. 9 kW gives transient,
// estimation and auxiliary-load margin below the 10 kW competition ceiling.
constexpr float DRIVE_POWER_SOFT_LIMIT_W = 9000.0f;
constexpr float DRIVETRAIN_EFFICIENCY = 0.92f;
constexpr float CONTROLLER_FEEDBACK_STALE_MS = 250.0f;
constexpr float CLUSTER_COMMAND_STALE_MS = 200.0f;
constexpr float PHASE_CURRENT_HARD_CUTOFF_A = 330.0f;
constexpr float CONTROLLER_DERATE_START_C = 75.0f;
constexpr float CONTROLLER_CUTOFF_C = 85.0f;
constexpr float MOTOR_DERATE_START_C = 100.0f;
constexpr float MOTOR_CUTOFF_C = 120.0f;
constexpr float PADDOCK_CURRENT_MAX_PER_MOTOR_A = 30.0f;
constexpr float PADDOCK_SPEED_LIMIT_MPS = 10.0f / 3.6f;
constexpr float PADDOCK_ENTRY_SPEED_MAX_MPS = 3.0f / 3.6f;
constexpr float TC_MIN_SPEED_MPS = 2.0f;
constexpr float TC_SLIP_START = 0.12f;
constexpr float TC_SLIP_FULL_CUT = 0.25f;
// Vehicle cannot enter Drive until the released throttle has stayed below
// this threshold for THROTTLE_ARM_CONSECUTIVE_TICKS scheduler passes.
constexpr float THROTTLE_ARM_MAX_PCT = 1.0f;
constexpr unsigned THROTTLE_ARM_CONSECUTIVE_TICKS = 30U;  // about 300 ms at 100 Hz
}  // namespace bringup

namespace confirmed {
// PCNT는 상승엣지만 센다. 네 바퀴 모두 휠 1회전당 상승엣지 48개.
constexpr float WSS_PULSES_PER_WHEEL_REV_FL = 48.0f;
constexpr float WSS_PULSES_PER_WHEEL_REV_FR = 48.0f;
constexpr float WSS_PULSES_PER_WHEEL_REV_RL = 48.0f;
constexpr float WSS_PULSES_PER_WHEEL_REV_RR = 48.0f;

constexpr float GEAR_RATIO = 3.72f;
constexpr float MOTOR_KT_NM_PER_A = 0.1266f;
constexpr float MOTOR_CONTINUOUS_CURRENT_MAX_A = 103.0f;
constexpr float CONTROL_PERIOD_S = 0.01f;  // 100 Hz
}  // namespace confirmed

namespace provisional {
// Initial Hall-throttle calibration. These values are deliberately
// conservative and MUST be replaced with the actual released/full ADC
// readings from the 5 Hz serial diagnostics before a driven test.
constexpr float THROTTLE_RAW_MIN = 620.0f;
constexpr float THROTTLE_RAW_MAX = 3720.0f;

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

// 48 PPR를 10 ms마다 직접 RPM으로 바꾸면 한 펄스 차이가 약 125 rpm이다.
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
