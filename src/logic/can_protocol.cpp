// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "can_protocol.h"

uint16_t torque_to_raw(float amps) {
    return (uint16_t)((amps + 3200.0f) * 10.0f + 0.5f);
}

float raw_to_torque(uint16_t raw) {
    return (float)raw / 10.0f - 3200.0f;
}
