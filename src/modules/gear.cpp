#include "modules/gear.h"

Gear gear_classify(uint16_t raw_adc, const GearCalib &calib) {
    if (raw_adc < calib.reverse_floor_adc) return Gear::Neutral;
    if (raw_adc < calib.drive_floor_adc) return Gear::Reverse;
    return Gear::Drive;
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
