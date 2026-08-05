// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include "modules/imu.h"   // for ImuRaw
// [LOCKED] Xsens MTi-320 over UART2 (VCU RX=GPIO16, TX=GPIO17), Xbus/MTData2.
// MTi-320 is RS-232, so these GPIOs must connect through a 3.3 V transceiver.

namespace imu_driver {
    bool   begin();        // resets parser and configures the ESP32 UART
    ImuRaw read();         // latest parsed sample: yaw_rate (deg/s) + accel (g)
    bool   stale();        // true until a complete sample arrives, or after 100 ms silence
}
