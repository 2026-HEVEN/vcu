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
