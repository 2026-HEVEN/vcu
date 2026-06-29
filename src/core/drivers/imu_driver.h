// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include "modules/imu.h"   // for ImuRaw
// [LOCKED] MPU-6050 over I2C (VCU SDA=GPIO21, SCL=GPIO22).

namespace imu_driver {
    bool   begin();        // returns false if device not found
    ImuRaw read();         // scaled gyro (deg/s) + accel (g)
}
