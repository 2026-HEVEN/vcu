// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include <cstdint>
// [SINGLE SOURCE OF TRUTH] Identical copy lives in the Cluster repo.
// Any edit here MUST be mirrored there. Owner: 김도현.

// --- CAN bus ---
constexpr uint32_t CAN_BITRATE = 250000;     // 250 kbps

// --- Node source addresses ---
constexpr uint8_t SA_VCU          = 0xD0;
constexpr uint8_t SA_CLUSTER      = 0xC0;
constexpr uint8_t SA_CONTROLLER_L = 0xEF;
constexpr uint8_t SA_CONTROLLER_R = 0xF0;
constexpr uint8_t SA_ENERGY_METER = 0x17;

// --- Torque command IDs (29-bit extended) ---
constexpr uint32_t CAN_ID_TORQUE_L = 0x0C01EFD0;
constexpr uint32_t CAN_ID_TORQUE_R = 0x0C01F0D0;

// --- Torque scaling: raw = (amps + 3200) * 10 ---
uint16_t torque_to_raw(float amps);
float    raw_to_torque(uint16_t raw);

// --- Cluster additions (mirror back into the VCU repo's can_protocol.h) ---
// MCU -> VCU feedback (Controller_L). Controller_R replaces SA 0xEF with 0xF0.
constexpr uint32_t CAN_ID_FB1_L = 0x1801D0EF;   // Part I: voltage/current/speed
constexpr uint32_t CAN_ID_FB2_L = 0x1802D0EF;   // Part II: temps/status/errors
// Cluster -> VCU command (config/reset). HEVEN-defined.
constexpr uint32_t CAN_ID_CLUSTER_CMD = 0x1801D0C0;

// Signal decoders (EZkontrol scaling)
float raw_to_voltage(uint16_t raw);   // 0.1 V/bit, offset 0
float raw_to_current(uint16_t raw);   // 0.1 A/bit, offset -3200 A
int   raw_to_temp(uint8_t raw);       // 1 C/bit, offset -40 C
int   raw_to_speed(uint16_t raw);     // 1 rpm/bit, offset -32000 rpm (VCU path)
