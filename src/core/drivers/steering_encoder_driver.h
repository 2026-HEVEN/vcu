// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include "modules/steering.h"   // for SteerRaw
// [LOCKED] Harness v5 slide potentiometer on GPIO25 (12-bit ADC).
// Samples are scaled to the existing 14-bit SteerRaw interface.

namespace steering_encoder_driver {
    bool     begin();
    SteerRaw read();   // 12-bit ADC scaled to 14-bit (0..16380)
}
