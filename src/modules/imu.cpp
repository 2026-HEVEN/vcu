#include "modules/imu.h"

// Xbus/MTData2 parsing and unit conversion happen in the driver
// (src/core/drivers/imu_driver.cpp). Map the current installation to the
// vehicle frame: X/Y are reversed, Z (yaw rate) is unchanged. Apply this
// mapping exactly once, before both TV and telemetry consume the values.
// This is not a stationary-offset or gravity correction. Revisit if the
// sensor mounting/output alignment changes to avoid double compensation.
ImuOutput imu_compute(const ImuRaw &raw) {
    return ImuOutput{ raw.yaw_rate, -raw.accel_x, -raw.accel_y };
}
