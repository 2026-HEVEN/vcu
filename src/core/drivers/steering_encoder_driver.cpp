#include "core/drivers/steering_encoder_driver.h"
#include <Arduino.h>
#include <Wire.h>

// [LOCKED] AS5600 RAW ANGLE registers 0x0C/0x0D (12-bit). Scaled to 14-bit for a
// common SteerRaw range. Replace with SPI transaction for AS5048A if used.
namespace {
    constexpr uint8_t AS5600_ADDR = 0x36;
    constexpr uint8_t REG_RAW_HI  = 0x0C;
}

namespace steering_encoder_driver {

bool begin() {
    Wire.begin(21, 22);
    Wire.beginTransmission(AS5600_ADDR);
    return Wire.endTransmission() == 0;
}

SteerRaw read() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_RAW_HI);
    Wire.endTransmission(false);
    Wire.requestFrom((int)AS5600_ADDR, 2);
    uint16_t hi = Wire.read(), lo = Wire.read();
    uint16_t raw12 = ((hi << 8) | lo) & 0x0FFF;   // 0..4095
    return SteerRaw{ (uint16_t)(raw12 << 2) };    // -> 0..16380 (14-bit scale)
}

} // namespace steering_encoder_driver
