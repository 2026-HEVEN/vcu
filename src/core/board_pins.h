#pragma once

// Physical VCU PCB / harness v5 pin map.
// Source of truth: Notion "하네스 설계 (v5)" and origin/GPIO-fixed.
namespace board_pins {
constexpr int CAN_RX = 16;
constexpr int CAN_TX = 17;

constexpr int THROTTLE_ADC = 32;
constexpr int BRAKE_DIGITAL = 33;
constexpr int GEAR_ADC = 27;

constexpr int WSS_FL = 36;
constexpr int WSS_FR = 39;
constexpr int WSS_RL = 34;
constexpr int WSS_RR = 35;

constexpr int STEERING_ADC = 25;

// Harness v5.1 reserves these, but acquisition/scaling and CAN IDs are not
// finalized. Do not read the floating inputs until the sensors are installed.
constexpr int LV_VOLTAGE_ADC_RESERVED = 13;
constexpr int AIR_RELAY_MONITOR_RESERVED = 14;
constexpr int BRAKE_PRESSURE_ADC_RESERVED = 26;

// Harness v5 lists the MTi UART pair as D22/D21. RX is listed first.
constexpr int IMU_RX = 22;
constexpr int IMU_TX = 21;
} // namespace board_pins
