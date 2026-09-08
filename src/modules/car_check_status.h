#pragma once
#include "car_check_protocol.h"

// Diagnostic-only: this function never changes a control command or a safety gate.
struct CarCheckStatusInput {
    bool tv_requested, regen_requested, paddock_requested, paddock_active;
    bool cluster_fresh, tv_pipeline_active, gains_enabled, imu_valid;
    bool speed_valid, speed_above_tv_min, output_allowed, test_override;
    bool regen_validated, brake_installed, bms_valid, brake_demand;
    bool longitudinal_regen_demand;
    float pack_soc, left_a, right_a;
    int direction_sign; // +1 Drive, -1 Reverse, 0 other
};
car_check::Control car_check_status_compute(const CarCheckStatusInput &in);
bool car_check_wheel_sample_valid(bool driver_ok, uint32_t delta, uint32_t dt_ms, float ppr);
