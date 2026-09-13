// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/steering.h"
#include "modules/realcar_calibration.h"
#include <cmath>

Unit steering_compute(const SteerRaw &raw, const SteerCalib &c) {
    float delta = (float)raw.counts - (float)c.center_counts;
    float u = (c.counts_per_unit > 0.0f) ? delta / c.counts_per_unit : 0.0f;
    if (c.invert) u = -u;
    return Unit(u);   // type clamps to -1..+1
}

bool steering_sample_valid(const SteerRaw &raw, float steering_unit,
                           bool sensor_installed) {
    return sensor_installed &&
        raw.counts >= realcar_cal::bringup::STEERING_RAW_VALID_MIN_COUNTS &&
        raw.counts <= realcar_cal::bringup::STEERING_RAW_VALID_MAX_COUNTS &&
        std::isfinite(steering_unit);
}
