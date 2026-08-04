// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "core/drivers/imu_driver.h"
#include "modules/imu_xbus.h"
#include <Arduino.h>

// [LOCKED] Xsens MTi-320 over ESP32 UART2, Xbus / MTData2 binary protocol.
// The MTi-320 electrical interface is RS-232: an external RS-232-to-3.3V-UART
// transceiver is required between the sensor and GPIO16/17.
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

    // No valid MTData2 frame for this long -> treat the sample as stale
    // (10x the 10ms imu_update() period; tolerates a few dropped frames).
    constexpr uint32_t STALE_TIMEOUT_MS = 100;

    ImuXbusParser parser;
    uint32_t last_valid_ms_ = 0;
    bool has_valid_sample_ = false;
}

namespace imu_driver {

bool begin() {
    parser.reset();
    last_valid_ms_ = 0;
    has_valid_sample_ = false;
    mti.begin(MTI_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
    // HardwareSerial cannot prove that the remote sensor is present. Runtime
    // health is established only after a complete MTData2 sample is parsed.
    return true;
}

ImuRaw read() {
    while (mti.available()) {
        if (parser.feed(static_cast<uint8_t>(mti.read()))) {
            last_valid_ms_ = millis();
            has_valid_sample_ = true;
        }
    }
    return parser.sample();
}

bool stale() {
    return !has_valid_sample_ ||
           static_cast<uint32_t>(millis() - last_valid_ms_) > STALE_TIMEOUT_MS;
}

} // namespace imu_driver
