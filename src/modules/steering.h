#pragma once
#include <cstdint>
#include "types.h"
// [FILL-IN] Raw sensor counts -> normalized steering. Calibration owned by team.
// 현재 실차는 D25의 12-bit ADC 슬라이드 포텐셔미터를 사용하고 드라이버가
// 기존 인터페이스와 맞도록 14-bit 범위로 스케일한다.

struct SteerRaw   { uint16_t counts; };                          // 12-bit ADC -> 0..16380
struct SteerCalib { uint16_t center_counts; float counts_per_unit; bool invert; };

Unit steering_compute(const SteerRaw &raw, const SteerCalib &c);
