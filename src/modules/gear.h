#pragma once
#include <cstdint>

// Values intentionally match the VCU -> Cluster status contract.
enum class Gear : uint8_t { Neutral = 0, Reverse = 1, Drive = 2, Park = 3 };

struct GearCalib {
    uint16_t neutral_floor_adc;
    uint16_t reverse_floor_adc;
    uint16_t drive_floor_adc;
    uint16_t reserved;
};

struct GearFilterState {
    Gear stable = Gear::Neutral;
    Gear candidate = Gear::Neutral;
    unsigned candidate_samples = 0;
};

Gear gear_classify(uint16_t raw_adc, const GearCalib &calib);
Gear gear_update(uint16_t raw_adc, const GearCalib &calib,
                 unsigned stable_samples, GearFilterState &state);
