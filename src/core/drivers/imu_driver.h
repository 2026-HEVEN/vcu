// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include "modules/imu.h"   // for ImuRaw
// [LOCKED] Xsens MTi-320 over UART2 (VCU RX=GPIO16, TX=GPIO17), Xbus/MTData2.

namespace imu_driver {
    bool   begin();        // returns true once UART is configured
    ImuRaw read();         // latest parsed sample: yaw_rate (deg/s) + accel (g)
}
