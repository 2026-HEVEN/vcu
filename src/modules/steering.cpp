// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/steering.h"

Unit steering_compute(const SteerRaw &raw, const SteerCalib &c) {
    float delta = (float)raw.counts - (float)c.center_counts;
    float u = (c.counts_per_unit > 0.0f) ? delta / c.counts_per_unit : 0.0f;
    if (c.invert) u = -u;
    return Unit(u);   // type clamps to -1..+1
}
