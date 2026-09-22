#pragma once

// Physical VCU PCB V3 pin map.
// Source of truth: Notion "PCB V3 설계" (2026-09-17).
namespace board_pins {
constexpr int CAN_RX = 23;
constexpr int CAN_TX = 22;

constexpr int STEERING_ADC = 36;
constexpr int BRAKE_PRESSURE_ADC = 39;
constexpr int THROTTLE_ADC = 34;
constexpr int BRAKE_ONOFF_ADC = 35;
constexpr int GEAR_ADC = 32;
constexpr int LV_VOLTAGE_ADC = 33;

constexpr int WSS_FL = 18;
constexpr int WSS_FR = 17;
constexpr int WSS_RL = 16;
constexpr int WSS_RR = 4;

constexpr int IMU_RX = 21;
constexpr int IMU_TX = 19;
} // namespace board_pins
