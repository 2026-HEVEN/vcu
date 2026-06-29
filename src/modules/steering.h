#pragma once
#include <cstdint>
#include "types.h"
// [FILL-IN] Absolute encoder counts -> normalized steering. Calibration owned by team.

struct SteerRaw   { uint16_t counts; };                          // 0..16383
struct SteerCalib { uint16_t center_counts; float counts_per_unit; bool invert; };

Unit steering_compute(const SteerRaw &raw, const SteerCalib &c);
