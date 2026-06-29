#pragma once
#include "modules/steering.h"   // for SteerRaw
// [LOCKED] Absolute magnetic encoder. Default: AS5600 over I2C.
// Swap to AS5048A (SPI) by reimplementing read() — interface stays the same.

namespace steering_encoder_driver {
    bool     begin();
    SteerRaw read();   // 14-bit absolute angle (0..16383)
}
