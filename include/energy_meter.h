#pragma once
#include <cstdint>

// Matches cluster/dev (8a5bfc6): EM Gateway, not EZkontrol's METER 0x17.
constexpr uint8_t SA_EM_GW = 0xC1;
constexpr uint32_t CAN_ID_EM_RECORD = 0x1CF5FFC1;
constexpr uint32_t CAN_ID_EM_SYNC = 0x1CF6FFC1;

struct EnergyMeterRecord {
    float hv_voltage_v = 0;
    float current_a = 0; // signed: discharge +, charge - (Cluster convention)
    float lv_voltage_v = 0;
    float cpu_temperature_c = 0;
    float power_w = 0; // signed DC power; never fabs() a charge sample
};
EnergyMeterRecord decode_em_record(const uint8_t data[8]);

struct EnergyMeterReceiver {
    EnergyMeterRecord record{};
    uint32_t last_rx_ms = 0;
    uint32_t rx_count = 0;
    uint32_t rejected_count = 0;
    bool seen = false;
    bool sample_valid = false;
    bool receive(uint32_t id, bool extended, bool rtr, uint8_t dlc,
                 const uint8_t *data, uint32_t now);
    bool fresh(uint32_t now, uint32_t stale_ms) const;
};
