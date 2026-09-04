// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include <cstdint>
#include "modules/imu.h"   // for ImuRaw
// [LOCKED] Xsens MTi-320 over UART2 (VCU RX=GPIO22, TX=GPIO21), Xbus/MTData2.

namespace imu_driver {
    struct Diagnostics {
        uint32_t rx_bytes;
        uint32_t valid_mtdata2_frames;
        uint32_t checksum_errors;
    };

    bool   begin();        // returns true once UART is configured
    ImuRaw read();         // latest parsed sample: yaw_rate (deg/s) + accel (g)
    bool   stale();        // true if no valid MTData2 frame recently (sensor dead/disconnected)
    Diagnostics diagnostics();
}
