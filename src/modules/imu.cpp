#include "modules/imu.h"

// Xbus/MTData2 parsing and unit conversion happen in the driver
// (src/core/drivers/imu_driver.cpp); the MTi-320 fuses roll/pitch/yaw/rate
// on-device, so this layer is a direct pass-through.
ImuOutput imu_compute(const ImuRaw &raw) {
    return ImuOutput{ raw.yaw_rate, raw.accel_x, raw.accel_y };
}
