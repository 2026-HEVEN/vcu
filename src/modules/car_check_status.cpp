#include "modules/car_check_status.h"
#include <cmath>
bool car_check_wheel_sample_valid(bool driver_ok,uint32_t delta,uint32_t dt_ms,float ppr) {
    return driver_ok && dt_ms>0 && dt_ms<=100 && std::isfinite(ppr) && ppr>0 &&
        float(delta)<=ppr*6000.0f*float(dt_ms)/60000.0f+1.0f;
}
car_check::Control car_check_status_compute(const CarCheckStatusInput &in) {
    using namespace car_check;
    Control o; o.supported=true;
    o.tv_requested=in.tv_requested; o.regen_requested=in.regen_requested;
    o.paddock_requested=in.paddock_requested; o.paddock_active=in.paddock_active;
    o.cluster_fresh=in.cluster_fresh; o.brake_installed=in.brake_installed;
    o.output_allowed=in.output_allowed && !in.test_override;
    if(!in.tv_requested) o.tv_block |= TV_REQUEST_OFF;
    if(!in.gains_enabled) o.tv_block |= TV_GAINS_ZERO;
    if(!in.imu_valid) o.tv_block |= TV_IMU_INVALID;
    if(!in.speed_valid) o.tv_block |= TV_SPEED_INVALID;
    if(!in.speed_above_tv_min) o.tv_block |= TV_LOW_SPEED;
    if(!in.output_allowed) o.tv_block |= TV_OUTPUT_BLOCKED;
    if(in.test_override) o.tv_block |= TV_TEST_OVERRIDE;
    o.tv_active=in.tv_pipeline_active && o.output_allowed && in.cluster_fresh;
    if(!in.regen_requested) o.regen_block |= REGEN_REQUEST_OFF;
    if(!in.regen_validated) o.regen_block |= REGEN_NOT_VALIDATED;
    if(!in.brake_installed) o.regen_block |= REGEN_NO_BRAKE_SENSOR;
    if(!in.bms_valid) o.regen_block |= REGEN_BMS_INVALID;
    if(!o.output_allowed) o.regen_block |= REGEN_OUTPUT_BLOCKED;
    if(!in.brake_demand) o.regen_block |= REGEN_NO_BRAKE_DEMAND;
    const bool soc_ok=std::isfinite(in.pack_soc) && in.pack_soc>=0 && in.pack_soc<0.95f;
    if(!soc_ok) o.regen_block |= REGEN_SOC_BLOCKED;
    // Report the known sign-loss defect as blocked, never as active regen.
    const bool signed_braking=in.direction_sign!=0 && std::isfinite(in.left_a) &&
        std::isfinite(in.right_a) && in.left_a*in.direction_sign<=0 &&
        in.right_a*in.direction_sign<=0 && (in.left_a!=0 || in.right_a!=0);
    if(in.longitudinal_regen_demand && !signed_braking) o.regen_block |= REGEN_DIRECTION_MISMATCH;
    o.regen_available=in.regen_requested && in.cluster_fresh && in.regen_validated &&
        in.brake_installed && in.bms_valid && soc_ok && o.output_allowed;
    o.regen_active=o.regen_available && in.brake_demand &&
        in.longitudinal_regen_demand && signed_braking;
    return o;
}
