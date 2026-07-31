// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "can_protocol.h"

uint16_t torque_to_raw(float amps) {
    return (uint16_t)((amps + 3200.0f) * 10.0f + 0.5f);
}

float raw_to_torque(uint16_t raw) {
    return (float)raw / 10.0f - 3200.0f;
}

float raw_to_voltage(uint16_t raw) { return (float)raw * 0.1f; }
float raw_to_current(uint16_t raw) { return (float)raw * 0.1f - 3200.0f; }
int   raw_to_temp(uint8_t raw)     { return (int)raw - 40; }
int   raw_to_speed(uint16_t raw)   { return (int)raw - 32000; }

uint16_t wheel_rpm_to_raw(float rpm) {
    if (rpm < 0.0f) return 0;
    if (rpm > 65535.0f) return 65535;
    return (uint16_t)(rpm + 0.5f);
}

uint16_t vehicle_speed_kph_to_raw(float kph) {
    if (kph < 0.0f) return 0;
    const float raw = kph * 10.0f;
    if (raw > 65535.0f) return 65535;
    return (uint16_t)(raw + 0.5f);
}

namespace {
void put_u16le(uint8_t *data, uint16_t value) {
    data[0] = (uint8_t)(value & 0xFF);
    data[1] = (uint8_t)((value >> 8) & 0xFF);
}
}

void encode_vcu_wheel_speeds(float fl_rpm, float fr_rpm,
                             float rl_rpm, float rr_rpm, uint8_t out[8]) {
    put_u16le(out + 0, wheel_rpm_to_raw(fl_rpm));
    put_u16le(out + 2, wheel_rpm_to_raw(fr_rpm));
    put_u16le(out + 4, wheel_rpm_to_raw(rl_rpm));
    put_u16le(out + 6, wheel_rpm_to_raw(rr_rpm));
}

void encode_vcu_vehicle_speed(float speed_kph, bool valid, uint8_t out[8]) {
    for (int i = 0; i < 8; ++i) out[i] = 0;
    put_u16le(out + 0, valid ? vehicle_speed_kph_to_raw(speed_kph) : 0);
    out[2] = valid ? 1 : 0;
}
