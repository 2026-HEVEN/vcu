// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include "modules/wheel_speed.h"    // for WssReading
#include "modules/vehicle_speed.h"  // for WHEEL_COUNT
// [LOCKED] PCNT-based wheel speed pulse counter, 4 channels. Hardware-only.
// 채널 인덱스는 vehicle_speed.h 의 WheelIdx(FL/FR/RL/RR) 순서를 따른다.

namespace wss_driver {
    void begin(int ch, int gpio);  // configure PCNT unit `ch` + glitch filter
    WssReading read(int ch);       // pulse delta + elapsed ms since last read
}
