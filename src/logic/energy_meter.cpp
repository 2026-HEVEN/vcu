#include "energy_meter.h"

namespace {
int32_t signed_le(const uint8_t *p) {
    const uint32_t raw = uint32_t(p[0]) | (uint32_t(p[1]) << 8);
    return raw >= 0x8000u ? int32_t(raw) - 65536 : int32_t(raw);
}
}

EnergyMeterRecord decode_em_record(const uint8_t data[8]) {
    EnergyMeterRecord out;
    out.hv_voltage_v = signed_le(data) * 0.1f;
    out.current_a = signed_le(data + 2) * 0.1f;
    out.lv_voltage_v = signed_le(data + 4) * 0.01f;
    out.cpu_temperature_c = signed_le(data + 6) * 0.01f;
    out.power_w = out.hv_voltage_v * out.current_a;
    return out;
}

bool EnergyMeterReceiver::receive(uint32_t id, bool extended, bool rtr,
                                  uint8_t dlc, const uint8_t *data, uint32_t now) {
    if (id != CAN_ID_EM_RECORD) return false; // SYNC never refreshes power
    if (!extended || rtr || dlc != 8 || data == nullptr) {
        ++rejected_count;
        sample_valid = false;
        return true;
    }
    record = decode_em_record(data);
    last_rx_ms = now;
    seen = true;
    ++rx_count;
    // Zero HV is displayed as a real measurement but cannot authorize driving.
    // Do not invent sensor sentinel meanings or a voltage calibration offset.
    sample_valid = record.hv_voltage_v > 0.0f;
    if (!sample_valid) ++rejected_count;
    return true;
}

bool EnergyMeterReceiver::fresh(uint32_t now, uint32_t stale_ms) const {
    return seen && sample_valid && uint32_t(now - last_rx_ms) <= stale_ms;
}
