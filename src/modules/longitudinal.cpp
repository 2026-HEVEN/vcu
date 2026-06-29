// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/longitudinal.h"

float longitudinal_compute(const LongInput &in) {
    // SAFE STUB: linear map. TODO(team-torquevectoring): real regen strategy —
    // e.g. in Efficiency mode prefer regen over friction brake; limit regen when
    // pack_soc is high to avoid overcharge.
    constexpr float DRIVE_MAX_A = 30.0f;
    constexpr float REGEN_MAX_A = 20.0f;
    float drive = in.throttle_pct / 100.0f * DRIVE_MAX_A;
    float regen = in.brake_pct    / 100.0f * REGEN_MAX_A;
    return drive - regen;
}
