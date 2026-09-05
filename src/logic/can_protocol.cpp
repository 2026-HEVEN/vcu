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

uint16_t motor_speed_to_raw(int rpm) {
    if (rpm < -32000) rpm = -32000;
    if (rpm >  32000) rpm =  32000;
    return (uint16_t)(rpm + 32000);
}

void encode_motor_control(float amps, int target_rpm, bool running,
                          uint8_t life, uint8_t out[8]) {
    for (int i = 0; i < 8; ++i) out[i] = 0;

    const uint16_t current_raw = torque_to_raw(amps);
    const uint16_t speed_raw = motor_speed_to_raw(target_rpm);
    out[0] = (uint8_t)(current_raw & 0xFF);
    out[1] = (uint8_t)((current_raw >> 8) & 0xFF);
    out[2] = (uint8_t)(speed_raw & 0xFF);
    out[3] = (uint8_t)((speed_raw >> 8) & 0xFF);
    out[4] = running ? 0x01 : 0x00;  // bit0 RUNNING, bit1=0 Torque Control
    out[7] = life;
}

float raw_to_voltage(uint16_t raw) { return (float)raw * 0.1f; }
float raw_to_current(uint16_t raw) { return (float)raw * 0.1f - 3200.0f; }
int   raw_to_temp(uint8_t raw)     { return (int)raw - 40; }
int   raw_to_speed(uint16_t raw)   { return (int)raw - 32000; }

ClusterCommandRequest decode_cluster_command(const uint8_t data[8]) {
    ClusterCommandRequest cmd;
    cmd.tv_enabled = (data[1] & 0x01) != 0;
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
uint16_t get_u16le(const uint8_t *data) {
    return (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
}

void put_u16le(uint8_t *data, uint16_t value) {
    data[0] = (uint8_t)(value & 0xFF);
    data[1] = (uint8_t)((value >> 8) & 0xFF);
}

void put_i16le(uint8_t *data, int16_t value) {
    put_u16le(data, (uint16_t)value);
}
}

ControllerFeedbackPart1 decode_controller_feedback_part1(const uint8_t data[8]) {
    ControllerFeedbackPart1 out;
    out.bus_voltage_v = raw_to_voltage(get_u16le(data + 0));
    out.bus_current_a = raw_to_current(get_u16le(data + 2));
    out.phase_current_a = raw_to_current(get_u16le(data + 4));
    out.motor_speed_rpm = raw_to_speed(get_u16le(data + 6));
    return out;
}

ControllerFeedbackPart2 decode_controller_feedback_part2(const uint8_t data[8]) {
    ControllerFeedbackPart2 out;
    out.controller_temp_c = raw_to_temp(data[0]);
    out.motor_temp_c = raw_to_temp(data[1]);
    out.running = (data[2] & 0x01) != 0;
    out.speed_mode = (data[2] & 0x02) != 0;
    out.error1 = data[3];
    out.error2 = data[4];
    out.error3 = data[5];
    out.life = data[7];
    return out;
}

ClusterBmsStatus decode_cluster_bms_status(const uint8_t data[8]) {
    ClusterBmsStatus out;
    out.valid = (data[0] & 0x01) != 0;
    out.ble_connected = (data[0] & 0x02) != 0;
    out.soc_pct = data[1] <= 100 ? data[1] : 100;
    out.pack_voltage_v = raw_to_voltage(get_u16le(data + 2));
    out.pack_current_a = raw_to_current(get_u16le(data + 4));
    out.temperature_c = raw_to_temp(data[6]);
    out.life = data[7];
    return out;
}

void encode_vcu_cluster_status(uint8_t gear, bool brake, bool hv_active,
                               bool soc_valid, uint8_t soc_pct,
                               bool throttle_valid, uint8_t throttle_pct,
                               uint8_t life,
                               uint8_t out[8]) {
    for (int i = 0; i < 8; ++i) out[i] = 0;
    out[0] = gear <= 3 ? gear : 0;
    out[1] = (brake ? 0x01 : 0x00) |
             (hv_active ? 0x02 : 0x00) |
             (soc_valid ? 0x04 : 0x00) |
             (throttle_valid ? 0x08 : 0x00);
    out[2] = soc_valid ? (soc_pct <= 100 ? soc_pct : 100) : 0;
    out[3] = throttle_valid ? (throttle_pct <= 100 ? throttle_pct : 100) : 0;
    out[7] = life;
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

void encode_vcu_imu(float yaw_rate_dps, float accel_x_g, float accel_y_g,
                    uint8_t out[8]) {
    for (int i = 0; i < 8; ++i) out[i] = 0;
    put_i16le(out + 0, telemetry_to_i16(yaw_rate_dps, 100.0f));
    put_i16le(out + 2, telemetry_to_i16(accel_x_g, 100.0f));
    put_i16le(out + 4, telemetry_to_i16(accel_y_g, 100.0f));
}
