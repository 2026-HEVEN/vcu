// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "core/drivers/imu_driver.h"
#include <Arduino.h>
#include <cstring>

// [LOCKED] Xsens MTi-320 over UART2, Xbus / MTData2 binary protocol.
//
// Frame:  FA  BID  MID  LEN  payload...  CS
//         sum(BID..CS) & 0xFF == 0
//
// MTData2 payload is a run of TLV groups: [DataID(2B,BE)][Len(1B)][Data(Len B)].
// We only pull the three groups the vehicle state cares about; everything
// else in the payload is skipped over using its own length.
//
// NOTE: LEN == 0xFF (Xbus "extended length", real length in next 2 bytes)
// is not supported — our enabled output (Euler + Accel + RateOfTurn, ~45B)
// never legitimately reaches it. Rather than read past payload_[], any frame
// reporting LEN == 0xFF is dropped immediately and the parser resyncs on the
// next 0xFA (see feed()/WAIT_LEN).
namespace {
    constexpr uint32_t MTI_BAUD  = 115200;
    constexpr int       PIN_RX   = 16;
    constexpr int       PIN_TX   = 17;
    HardwareSerial &mti = Serial2;

    constexpr uint8_t  PREAMBLE    = 0xFA;
    constexpr uint8_t  MID_MTDATA2 = 0x36;

    constexpr uint16_t XDI_EULER_ANGLES = 0x2030;  // roll,pitch,yaw (deg)      - unused downstream
    constexpr uint16_t XDI_ACCELERATION = 0x4020;  // x,y,z (m/s^2)
    constexpr uint16_t XDI_RATE_OF_TURN = 0x8020;  // x,y,z (rad/s)

    constexpr float MPS2_TO_G   = 1.0f / 9.80665f;
    constexpr float RAD_TO_DEG  = 57.29578f;

    // No valid MTData2 frame for this long -> treat the sample as stale
    // (10x the 10ms imu_update() period; tolerates a few dropped frames).
    constexpr uint32_t STALE_TIMEOUT_MS = 100;

    enum class State { WAIT_PRE, WAIT_BID, WAIT_MID, WAIT_LEN, WAIT_PAYLOAD, WAIT_CS };

    State    state_        = State::WAIT_PRE;
    uint8_t  mid_          = 0;
    uint8_t  len_          = 0;
    uint8_t  idx_          = 0;
    uint8_t  checksum_     = 0;
    uint8_t  payload_[254];

    ImuRaw   latest_{ 0.0f, 0.0f, 0.0f };
    uint32_t last_valid_ms_ = 0;  // millis() of last checksum-valid MTData2 frame; 0 = none yet

    float be_float(const uint8_t *p) {
        uint8_t sw[4] = { p[3], p[2], p[1], p[0] };  // MTi floats are big-endian
        float f;
        memcpy(&f, sw, 4);
        return f;
    }

    void handle_mtdata2_payload() {
        uint8_t i = 0;
        while (i + 3 <= len_) {
            uint16_t xdi  = (payload_[i] << 8) | payload_[i + 1];
            uint8_t  dlen = payload_[i + 2];
            const uint8_t *data = &payload_[i + 3];
            if (i + 3 + dlen > len_) break;  // malformed group, stop

            if (xdi == XDI_ACCELERATION && dlen == 12) {
                latest_.accel_x = be_float(data) * MPS2_TO_G;
                latest_.accel_y = be_float(data + 4) * MPS2_TO_G;
            } else if (xdi == XDI_RATE_OF_TURN && dlen == 12) {
                latest_.yaw_rate = be_float(data + 8) * RAD_TO_DEG;  // z axis
            }
            // XDI_EULER_ANGLES intentionally skipped: VehicleState has no
            // attitude fields to put it in.

            i += 3 + dlen;
        }
    }

    void feed(uint8_t b) {
        switch (state_) {
            case State::WAIT_PRE:
                if (b == PREAMBLE) state_ = State::WAIT_BID;
                break;
            case State::WAIT_BID:
                checksum_ = b;
                state_ = State::WAIT_MID;
                break;
            case State::WAIT_MID:
                mid_ = b; checksum_ += b;
                state_ = State::WAIT_LEN;
                break;
            case State::WAIT_LEN:
                len_ = b; checksum_ += b; idx_ = 0;
                if (len_ == 0)         state_ = State::WAIT_CS;
                else if (len_ == 0xFF) state_ = State::WAIT_PRE;  // unsupported extended length; drop, resync
                else                   state_ = State::WAIT_PAYLOAD;
                break;
            case State::WAIT_PAYLOAD:
                payload_[idx_++] = b; checksum_ += b;
                if (idx_ >= len_) state_ = State::WAIT_CS;
                break;
            case State::WAIT_CS:
                checksum_ += b;
                if (checksum_ == 0 && mid_ == MID_MTDATA2) {
                    handle_mtdata2_payload();
                    last_valid_ms_ = millis();
                }
                state_ = State::WAIT_PRE;
                break;
        }
    }
}

namespace imu_driver {

bool begin() {
    mti.begin(MTI_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
    return true;
}

ImuRaw read() {
    while (mti.available()) feed((uint8_t)mti.read());
    return latest_;  // most recent fully-validated MTData2 sample
}

bool stale() {
    // last_valid_ms_ == 0 (no frame ever received) also reads as stale.
    return millis() - last_valid_ms_ > STALE_TIMEOUT_MS;
}

} // namespace imu_driver
