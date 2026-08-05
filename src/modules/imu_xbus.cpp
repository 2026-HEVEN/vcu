#include "modules/imu_xbus.h"

#include <cmath>
#include <cstring>

namespace {
constexpr uint8_t PREAMBLE = 0xFA;
constexpr uint8_t MID_MTDATA2 = 0x36;
constexpr uint16_t XDI_ACCELERATION = 0x4020;
constexpr uint16_t XDI_RATE_OF_TURN = 0x8020;
constexpr float MPS2_TO_G = 1.0f / 9.80665f;
constexpr float RADPS_TO_DEGPS = 57.2957795f;

float read_be_float(const uint8_t *bytes) {
    const uint8_t little_endian[4] = {
        bytes[3], bytes[2], bytes[1], bytes[0]
    };
    float value = 0.0f;
    std::memcpy(&value, little_endian, sizeof(value));
    return value;
}
} // namespace

void ImuXbusParser::reset() {
    state_ = State::WaitPreamble;
    message_id_ = 0;
    length_ = 0;
    index_ = 0;
    checksum_ = 0;
    latest_ = {0.0f, 0.0f, 0.0f};
    has_sample_ = false;
}

bool ImuXbusParser::parse_mtdata2() {
    ImuRaw candidate = latest_;
    bool has_acceleration = false;
    bool has_rate_of_turn = false;

    std::size_t offset = 0;
    while (offset + 3U <= length_) {
        const uint16_t data_id =
            (static_cast<uint16_t>(payload_[offset]) << 8U) |
            payload_[offset + 1U];
        const std::size_t data_length = payload_[offset + 2U];
        const std::size_t data_start = offset + 3U;
        if (data_start + data_length > length_) return false;

        const uint8_t *data = &payload_[data_start];
        if (data_id == XDI_ACCELERATION && data_length == 12U) {
            candidate.accel_x = read_be_float(data) * MPS2_TO_G;
            candidate.accel_y = read_be_float(data + 4U) * MPS2_TO_G;
            has_acceleration = std::isfinite(candidate.accel_x) &&
                               std::isfinite(candidate.accel_y);
        } else if (data_id == XDI_RATE_OF_TURN && data_length == 12U) {
            candidate.yaw_rate = read_be_float(data + 8U) * RADPS_TO_DEGPS;
            has_rate_of_turn = std::isfinite(candidate.yaw_rate);
        }

        offset = data_start + data_length;
    }

    if (offset != length_ || !has_acceleration || !has_rate_of_turn) {
        return false;
    }

    latest_ = candidate;
    has_sample_ = true;
    return true;
}

bool ImuXbusParser::feed(uint8_t byte) {
    switch (state_) {
        case State::WaitPreamble:
            if (byte == PREAMBLE) state_ = State::WaitBusId;
            break;
        case State::WaitBusId:
            checksum_ = byte;
            state_ = State::WaitMessageId;
            break;
        case State::WaitMessageId:
            message_id_ = byte;
            checksum_ += byte;
            state_ = State::WaitLength;
            break;
        case State::WaitLength:
            length_ = byte;
            checksum_ += byte;
            index_ = 0;
            if (length_ == 0U) {
                state_ = State::WaitChecksum;
            } else if (length_ == 0xFFU) {
                // Extended-length frames are deliberately unsupported. The
                // required two-vector output is well below 255 bytes.
                state_ = State::WaitPreamble;
            } else {
                state_ = State::WaitPayload;
            }
            break;
        case State::WaitPayload:
            payload_[index_++] = byte;
            checksum_ += byte;
            if (index_ == length_) state_ = State::WaitChecksum;
            break;
        case State::WaitChecksum: {
            checksum_ += byte;
            const bool complete = checksum_ == 0U &&
                                  message_id_ == MID_MTDATA2 &&
                                  parse_mtdata2();
            state_ = State::WaitPreamble;
            return complete;
        }
    }
    return false;
}
