// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/imu.h"

ImuOutput imu_compute(const ImuRaw &raw, ImuFilterState &s) {
    constexpr float ALPHA = 0.2f;   // low-pass weight on new sample
    if (!s.seeded) { s.yaw_rate_lp = raw.gyro_z; s.seeded = true; }
    else           { s.yaw_rate_lp = ALPHA * raw.gyro_z + (1.0f - ALPHA) * s.yaw_rate_lp; }
    return ImuOutput{ s.yaw_rate_lp, raw.accel_x, raw.accel_y };
}
