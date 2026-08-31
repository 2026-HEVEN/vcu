// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "core/drivers/steering_encoder_driver.h"
#include "core/board_pins.h"
#include <Arduino.h>

// Harness v5 uses a slide potentiometer on GPIO25 instead of an I2C encoder.
// Keep the existing 14-bit SteerRaw contract by scaling the 12-bit ADC sample.

namespace steering_encoder_driver {

bool begin() {
    pinMode(board_pins::STEERING_ADC, INPUT);
    return true;
}

SteerRaw read() {
    const uint16_t raw12 = static_cast<uint16_t>(analogRead(board_pins::STEERING_ADC));
    return SteerRaw{ static_cast<uint16_t>(raw12 << 2) };
}

} // namespace steering_encoder_driver
