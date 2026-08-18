#pragma once
#include <cstdint>
#include "types.h"
// [FILL-IN] Raw sensor counts -> normalized steering. Calibration owned by team.
// 센서 무관: AS5600 엔코더 14-bit이든 슬라이드 포텐 ADC 12-bit이든 선형맵 동일.

struct SteerRaw   { uint16_t counts; };                          // 엔코더 0..16383 / ADC 0..4095
struct SteerCalib { uint16_t center_counts; float counts_per_unit; bool invert; };

Unit steering_compute(const SteerRaw &raw, const SteerCalib &c);
