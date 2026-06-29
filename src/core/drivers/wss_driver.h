#pragma once
#include "modules/wheel_speed.h"   // for WssReading
// [LOCKED] PCNT-based wheel speed pulse counter. Hardware-only.

namespace wss_driver {
    void begin(int gpio);          // configure PCNT unit + glitch filter
    WssReading read();             // pulse delta + elapsed ms since last read
}
