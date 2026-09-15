#pragma once
#include <cstdint>
#include "types.h"
// Pure analog brake-pressure conversion. Hardware acquisition remains in the
// core; this contract is deterministic and host-testable.
struct BrakeInput { int raw_adc; };            // ESP32 12-bit ADC, 0..4095

struct BrakeCalib {
    uint16_t zero_adc;                          // measured at 0 bar
    uint16_t full_scale_adc;                    // measured at full_scale_bar
    uint16_t valid_min_adc;                     // short-to-GND/open detection
    uint16_t valid_max_adc;
    float full_scale_bar;
    float sample_period_s;                      // brake task cadence
    float filter_time_constant_s;               // first-order low-pass tau
    float active_on_threshold_pct;              // released -> pressed
    float active_off_threshold_pct;             // pressed -> released
};

struct BrakeFilterState {
    float filtered_adc = 0.0f;
    bool initialized = false;
    bool active = false;
};

struct BrakeOutput {
    Pct0to100 pct;
    float filtered_adc = 0.0f;
    float pressure_bar = 0.0f;
    bool active = false;
    bool valid = false;
};

BrakeOutput brake_compute(const BrakeInput &in, const BrakeCalib &calib,
                          BrakeFilterState &state);
