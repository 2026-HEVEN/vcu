#include "modules/gear.h"

namespace {
uint16_t distance(uint16_t a, uint16_t b) {
    return a > b ? (uint16_t)(a - b) : (uint16_t)(b - a);
}
}

Gear gear_classify(uint16_t raw_adc, const GearCalib &calib) {
    Gear best = Gear::Neutral;
    uint16_t best_distance = calib.tolerance;
    bool matched = false;

    const uint16_t dn = distance(raw_adc, calib.neutral_adc);
    if (dn <= best_distance) {
        best = Gear::Neutral;
        best_distance = dn;
        matched = true;
    }
    const uint16_t dr = distance(raw_adc, calib.reverse_adc);
    if (dr < best_distance) {
        best = Gear::Reverse;
        best_distance = dr;
        matched = true;
    }
    const uint16_t dd = distance(raw_adc, calib.drive_adc);
    if (dd < best_distance) {
        best = Gear::Drive;
        matched = true;
    }
    // An out-of-window or wire-fault value is fail-safe Neutral.
    return matched ? best : Gear::Neutral;
}

Gear gear_update(uint16_t raw_adc, const GearCalib &calib,
                 unsigned stable_samples, GearFilterState &state) {
    const Gear next = gear_classify(raw_adc, calib);
    if (next != state.candidate) {
        state.candidate = next;
        state.candidate_samples = 1;
    } else if (state.candidate_samples < stable_samples) {
        ++state.candidate_samples;
    }

    if (stable_samples == 0 || state.candidate_samples >= stable_samples) {
        state.stable = state.candidate;
    }
    return state.stable;
}
