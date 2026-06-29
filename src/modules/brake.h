#pragma once
#include "types.h"
// [FILL-IN] Pure brake conversion.

struct BrakeInput  { int raw_adc; };          // 0..4095
struct BrakeOutput { Pct0to100 pct; bool active; };

BrakeOutput brake_compute(const BrakeInput &in);
