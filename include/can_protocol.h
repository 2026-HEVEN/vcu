// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include <cstdint>
// Wire-level IDs and byte layouts are shared with the Cluster repo. VCU-only
// decoders/encoders below need not be copied, but any on-bus contract change
// must be coordinated with Cluster and Monolith. Owner: 김도현.

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
uint16_t motor_speed_to_raw(int rpm);

// VCU -> EZkontrol normal control frame (8 bytes):
//   byte0..1 target phase current, byte2..3 target speed,
//   byte4 bit0 RUNNING/HALTED, byte7 life counter.
// Handshake frames (all 0xAA) are deliberately handled separately.
void encode_motor_control(float amps, int target_rpm, bool running,
                          uint8_t life, uint8_t out[8]);

// --- Cluster additions (mirror back into the VCU repo's can_protocol.h) ---
// MCU -> VCU feedback (Controller_L). Controller_R replaces SA 0xEF with 0xF0.
constexpr uint32_t CAN_ID_FB1_L = 0x1801D0EF;   // Part I: voltage/current/speed
constexpr uint32_t CAN_ID_FB2_L = 0x1802D0EF;   // Part II: temps/status/errors
constexpr uint32_t CAN_ID_FB1_R = 0x1801D0F0;   // Part I, Controller_R
constexpr uint32_t CAN_ID_FB2_R = 0x1802D0F0;   // Part II, Controller_R
// Cluster -> VCU command. The physical/UI label "TC" means TV enable in the
// current HEVEN project contract.
constexpr uint32_t CAN_ID_CLUSTER_CMD = 0x1801D0C0;
// VCU -> Cluster confirmed vehicle state.
constexpr uint32_t CAN_ID_VCU_CLUSTER_STATUS = 0x1801C0D0;
// VCU -> Cluster/TMA-1 single vehicle speed. Byte 0..1 contains km/h x 10,
// byte 2 is valid flag (1=valid, 0=invalid), byte 3..7 reserved zero.
constexpr uint32_t CAN_ID_VCU_VEHICLE_SPEED = 0x1803C0D0;
constexpr uint32_t CAN_ID_VCU_STEERING = 0x1804C0D0;
constexpr uint32_t CAN_ID_VCU_IMU = 0x1805C0D0;
// Cluster -> logger BMS summary. VCU may observe this for diagnostics only;
// the BLE path is not an authoritative safety input.
constexpr uint32_t CAN_ID_CLUSTER_BMS_STATUS = 0x18F3FFC0;

struct ClusterCommandRequest {
    bool tv_enabled = false;
    bool regen_auto_enabled = false;
    bool debug_enabled = false;
    bool paddock_request = false;
};

// Cluster -> VCU command frame (0x1801D0C0):
// byte1 bit0=TC-labelled TV enable, bit1=Regen Auto, bit2=reserved, bit3=Debug,
// byte2 bit0=Paddock. Debug is kept as a request bit even if VCU ignores it.
ClusterCommandRequest decode_cluster_command(const uint8_t data[8]);

struct ControllerFeedbackPart1 {
    float bus_voltage_v = 0.0f;
    float bus_current_a = 0.0f;
    float phase_current_a = 0.0f;
    int motor_speed_rpm = 0;
};

struct ControllerFeedbackPart2 {
    int controller_temp_c = -40;
    int motor_temp_c = -40;
    bool running = false;
    bool speed_mode = false;
    uint8_t error1 = 0;
    uint8_t error2 = 0;
    uint8_t error3 = 0;
    uint8_t life = 0;
    bool any_fault() const { return error1 != 0 || error2 != 0 || error3 != 0; }
};

struct ClusterBmsStatus {
    bool valid = false;
    bool ble_connected = false;
    uint8_t soc_pct = 0;
    float pack_voltage_v = 0.0f;
    float pack_current_a = 0.0f;
    int temperature_c = -40;
    uint8_t life = 0;
};

ControllerFeedbackPart1 decode_controller_feedback_part1(const uint8_t data[8]);
ControllerFeedbackPart2 decode_controller_feedback_part2(const uint8_t data[8]);
ClusterBmsStatus decode_cluster_bms_status(const uint8_t data[8]);

// VCU -> Cluster status: byte0 gear (0=N,1=R,2=D,3=P), byte1 bit0 brake,
// bit1 HV active, bit2 SOC valid, byte2 SOC %, byte7 life counter.
void encode_vcu_cluster_status(uint8_t gear, bool brake, bool hv_active,
                               bool soc_valid, uint8_t soc_pct, uint8_t life,
                               uint8_t out[8]);
// VCU -> Cluster/TMA-1 single vehicle speed frame (0x1803C0D0). HEVEN-defined.
uint16_t vehicle_speed_kph_to_raw(float kph);
void encode_vcu_vehicle_speed(float speed_kph, bool valid, uint8_t out[8]);
int16_t telemetry_to_i16(float value, float scale);
void encode_vcu_steering(float steering_unit, uint8_t out[8]);
void encode_vcu_imu(float yaw_rate_dps, float accel_x_g, float accel_y_g,
                    uint8_t out[8]);

// Signal decoders (EZkontrol scaling)
float raw_to_voltage(uint16_t raw);   // 0.1 V/bit, offset 0
float raw_to_current(uint16_t raw);   // 0.1 A/bit, offset -3200 A
int   raw_to_temp(uint8_t raw);       // 1 C/bit, offset -40 C
int   raw_to_speed(uint16_t raw);     // 1 rpm/bit, offset -32000 rpm (VCU path)
