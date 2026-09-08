#ifndef HEVEN_CAR_CHECK_PROTOCOL_H
#define HEVEN_CAR_CHECK_PROTOCOL_H
#include <cstdint>

// HEVEN Car Check wire contract v1. Keep this file identical in VCU/Cluster.
// All frames: extended CAN, DLC=8, 250 kbit/s, nominal 50 ms.
// Existing steering/IMU numeric bytes are unchanged. See docs/CAR_CHECK_CAN.md.
namespace car_check {
constexpr uint32_t STEERING_ID = 0x1804C0D0;
constexpr uint32_t IMU_ID = 0x1805C0D0;
constexpr uint32_t WHEELS_ID = 0x1806C0D0;
constexpr uint32_t CONTROL_ID = 0x1807C0D0;
constexpr uint32_t PERIOD_MS = 50;
constexpr uint32_t STALE_MS = 300;
constexpr uint8_t VERSION = 1;
constexpr uint8_t VALIDITY_PRESENT = 0x80;
constexpr uint16_t INVALID_WHEEL = 0xFFFF;

struct Steering {
    float unit = 0.0f; // normalized -1..+1, NOT degrees
    bool valid = false;
    bool validity_present = false; // false for legacy sender
    uint8_t life = 0;
};
struct Imu {
    float yaw_dps = 0.0f;
    float ax_g = 0.0f;
    float ay_g = 0.0f;
    bool yaw_valid = false;
    bool accel_valid = false;
    bool validity_present = false;
    uint8_t life = 0;
};
struct Wheels {
    float kph[4]{}; // FL, FR, RL, RR; unsigned speed magnitude
    bool valid[4]{};
};
enum TvBlock : uint8_t {
    TV_REQUEST_OFF=1, TV_GAINS_ZERO=2, TV_IMU_INVALID=4,
    TV_SPEED_INVALID=8, TV_LOW_SPEED=16, TV_OUTPUT_BLOCKED=32,
    TV_TEST_OVERRIDE=64
};
enum RegenBlock : uint8_t {
    REGEN_REQUEST_OFF=1, REGEN_NOT_VALIDATED=2, REGEN_NO_BRAKE_SENSOR=4,
    REGEN_BMS_INVALID=8, REGEN_OUTPUT_BLOCKED=16, REGEN_NO_BRAKE_DEMAND=32,
    REGEN_SOC_BLOCKED=64, REGEN_DIRECTION_MISMATCH=128
};
struct Control {
    bool supported = false;
    bool tv_requested = false;
    bool regen_requested = false;
    bool paddock_requested = false;
    bool tv_active = false; // selected TV pipeline AND current output permission
    bool regen_available = false; // diagnostic permission, NOT BMS charge authority
    bool regen_active = false; // opposing signed command, NOT measured energy recovery
    bool paddock_active = false;
    bool output_allowed = false;
    bool cluster_fresh = false;
    bool brake_installed = false;
    uint8_t tv_block = 0;
    uint8_t regen_block = 0;
    uint8_t life = 0;
};

void encode_steering(const Steering &in, uint8_t out[8]);
void encode_imu(const Imu &in, uint8_t out[8]);
void encode_wheels(const Wheels &in, uint8_t out[8]);
void encode_control(const Control &in, uint8_t out[8]);
Steering decode_steering(const uint8_t data[8]);
Imu decode_imu(const uint8_t data[8]);
Wheels decode_wheels(const uint8_t data[8]);
Control decode_control(const uint8_t data[8]);

enum class Quality : uint8_t { Missing, Stale, Unsupported, Invalid, Valid };
struct Reception {
    bool seen = false;
    uint32_t last_ms = 0;
    void received(uint32_t now) { seen = true; last_ms = now; }
    Quality quality(uint32_t now, bool valid, bool supported=true) const {
        if (!seen) return Quality::Missing;
        if (uint32_t(now-last_ms) > STALE_MS) return Quality::Stale;
        if (!supported) return Quality::Unsupported;
        return valid ? Quality::Valid : Quality::Invalid;
    }
};
inline const char *quality_label(Quality q) {
    switch (q) {
    case Quality::Missing: return "-";
    case Quality::Stale: return "STALE";
    case Quality::Unsupported: return "UNKNOWN";
    case Quality::Invalid: return "INVALID";
    default: return "OK";
    }
}
} // namespace car_check
#endif
