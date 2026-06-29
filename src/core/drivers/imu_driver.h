#pragma once
#include "modules/imu.h"   // for ImuRaw
// [LOCKED] MPU-6050 over I2C (VCU SDA=GPIO21, SCL=GPIO22).

namespace imu_driver {
    bool   begin();        // returns false if device not found
    ImuRaw read();         // scaled gyro (deg/s) + accel (g)
}
