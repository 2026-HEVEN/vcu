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

ClusterCommandRequest decode_cluster_command(const uint8_t data[8]) {
    ClusterCommandRequest cmd;
    cmd.tc_enabled = (data[1] & 0x01) != 0;
    cmd.regen_auto_enabled = (data[1] & 0x02) != 0;
    cmd.debug_enabled = (data[1] & 0x08) != 0;
    cmd.paddock_request = (data[2] & 0x01) != 0;
    return cmd;
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

void put_i16le(uint8_t *data, int16_t value) {
    data[0] = (uint8_t)((uint16_t)value & 0xFF);
    data[1] = (uint8_t)(((uint16_t)value >> 8) & 0xFF);
}
}

void encode_vcu_vehicle_speed(float speed_kph, bool valid, uint8_t out[8]) {
    for (int i = 0; i < 8; ++i) out[i] = 0;
    put_u16le(out + 0, valid ? vehicle_speed_kph_to_raw(speed_kph) : 0);
    out[2] = valid ? 1 : 0;
}

int16_t telemetry_to_i16(float value, float scale) {
    const float raw = value * scale;
    if (raw > 32767.0f) return 32767;
    if (raw < -32768.0f) return -32768;
    return (int16_t)(raw >= 0.0f ? raw + 0.5f : raw - 0.5f);
}

void encode_vcu_steering(float steering_unit, uint8_t out[8]) {
    for (int i = 0; i < 8; ++i) out[i] = 0;
    put_i16le(out + 0, telemetry_to_i16(steering_unit, 1000.0f));
}

void encode_vcu_imu(float yaw_rate_dps, float accel_x_g, float accel_y_g, uint8_t out[8]) {
    for (int i = 0; i < 8; ++i) out[i] = 0;
    put_i16le(out + 0, telemetry_to_i16(yaw_rate_dps, 100.0f));
    put_i16le(out + 2, telemetry_to_i16(accel_x_g, 100.0f));
    put_i16le(out + 4, telemetry_to_i16(accel_y_g, 100.0f));
}
