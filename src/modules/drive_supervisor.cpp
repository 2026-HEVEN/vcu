#include "modules/drive_supervisor.h"
#include <cmath>

namespace {
float clamp01(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

float positive(float value) { return value > 0.0f ? value : 0.0f; }

float temperature_scale(float value, float derate_start, float cutoff) {
    if (value <= derate_start) return 1.0f;
    if (value >= cutoff || cutoff <= derate_start) return 0.0f;
    return (cutoff - value) / (cutoff - derate_start);
}

float tc_scale(float driven_rpm, float reference_rpm,
               float slip_start, float slip_full_cut) {
    if (reference_rpm <= 1.0f) return 1.0f;
    const float slip = (driven_rpm - reference_rpm) / reference_rpm;
    if (slip <= slip_start) return 1.0f;
    if (slip >= slip_full_cut || slip_full_cut <= slip_start) return 0.0f;
    return (slip_full_cut - slip) / (slip_full_cut - slip_start);
}

void scale_positive(float &value, float scale) {
    if (value > 0.0f) value *= scale;
}

void scale_all(float &value, float scale) { value *= scale; }
}

DriveSupervisorOutput drive_supervisor_compute(
    const DriveSupervisorInput &in, const DriveSupervisorParams &params) {
    DriveSupervisorOutput out;
    out.left_a = in.requested_left_a;
    out.right_a = in.requested_right_a;

    // Use absolute controller DC powers until the real-car bus-current sign
    // convention is verified. This is conservative for the 10 kW ceiling.
    out.measured_bus_power_w =
        std::fabs(in.bus_voltage_left_v * in.bus_current_left_a) +
        std::fabs(in.bus_voltage_right_v * in.bus_current_right_a);

    if (!in.controller_feedback_fresh || in.controller_fault) {
        out.left_a = 0.0f;
        out.right_a = 0.0f;
        out.controller_blocked = true;
        return out;
    }

    float thermal_scale = 1.0f;
    const float thermal_candidates[] = {
        temperature_scale(in.controller_temp_left_c,
                          params.controller_derate_start_c,
                          params.controller_cutoff_c),
        temperature_scale(in.controller_temp_right_c,
                          params.controller_derate_start_c,
                          params.controller_cutoff_c),
        temperature_scale(in.motor_temp_left_c,
                          params.motor_derate_start_c,
                          params.motor_cutoff_c),
        temperature_scale(in.motor_temp_right_c,
                          params.motor_derate_start_c,
                          params.motor_cutoff_c),
    };
    for (float candidate : thermal_candidates) {
        if (candidate < thermal_scale) thermal_scale = candidate;
    }
    thermal_scale = clamp01(thermal_scale);
    if (thermal_scale < 1.0f) {
        scale_all(out.left_a, thermal_scale);
        scale_all(out.right_a, thermal_scale);
        out.thermal_limited = true;
    }

    if (in.paddock_active) {
        if (out.left_a > params.paddock_current_per_motor_a)
            out.left_a = params.paddock_current_per_motor_a;
        if (out.right_a > params.paddock_current_per_motor_a)
            out.right_a = params.paddock_current_per_motor_a;
        if (in.vehicle_speed_mps >= params.paddock_speed_limit_mps) {
            if (out.left_a > 0.0f) out.left_a = 0.0f;
            if (out.right_a > 0.0f) out.right_a = 0.0f;
        }
        out.paddock_limited = true;
    }

    if (in.tc_enabled && in.vehicle_speed_valid &&
        in.vehicle_speed_mps >= params.tc_min_speed_mps) {
        const float front_reference = 0.5f * (in.wheel_rpm_fl + in.wheel_rpm_fr);
        const float left_scale = clamp01(tc_scale(
            in.wheel_rpm_rl, front_reference,
            params.tc_slip_start, params.tc_slip_full_cut));
        const float right_scale = clamp01(tc_scale(
            in.wheel_rpm_rr, front_reference,
            params.tc_slip_start, params.tc_slip_full_cut));
        if (left_scale < 1.0f || right_scale < 1.0f) {
            scale_positive(out.left_a, left_scale);
            scale_positive(out.right_a, right_scale);
            out.traction_limited = true;
        }
    }

    constexpr float TWO_PI_OVER_60 = 0.104719755f;
    const float efficiency = params.drivetrain_efficiency > 0.05f
        ? params.drivetrain_efficiency : 1.0f;
    out.estimated_input_power_w =
        (positive(out.left_a) * params.motor_kt_nm_per_a *
             std::fabs((float)in.motor_rpm_left) * TWO_PI_OVER_60 +
         positive(out.right_a) * params.motor_kt_nm_per_a *
             std::fabs((float)in.motor_rpm_right) * TWO_PI_OVER_60) /
        efficiency;

    float governing_power = out.measured_bus_power_w;
    if (out.estimated_input_power_w > governing_power)
        governing_power = out.estimated_input_power_w;

    float power_scale = 1.0f;
    if (params.power_soft_limit_w > 0.0f &&
        governing_power > params.power_soft_limit_w) {
        power_scale = clamp01(params.power_soft_limit_w / governing_power);
        scale_positive(out.left_a, power_scale);
        scale_positive(out.right_a, power_scale);
        out.power_limited = true;
    }
    out.applied_scale = thermal_scale * power_scale;
    return out;
}
