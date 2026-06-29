#include "core/drivers/imu_driver.h"
#include <Arduino.h>
#include <MPU6050.h>

// [LOCKED] MPU-6050 wrapper.
namespace {
    MPU6050 mpu_;
    constexpr float GYRO_LSB  = 131.0f;    // +-250 deg/s range
    constexpr float ACCEL_LSB = 16384.0f;  // +-2 g range
}

namespace imu_driver {

bool begin() {
    Wire.begin(21, 22);
    mpu_.initialize();
    return mpu_.testConnection();
}

ImuRaw read() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu_.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    return ImuRaw{ gz / GYRO_LSB, ax / ACCEL_LSB, ay / ACCEL_LSB };
}

} // namespace imu_driver
