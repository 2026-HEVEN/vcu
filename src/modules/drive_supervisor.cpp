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

void scale_positive(float &value, float scale) {
    if (value > 0.0f) value *= scale;
}

void scale_all(float &value, float scale) { value *= scale; }

float positive_limit_scale(float measured, float limit) {
    if (limit <= 0.0f || measured <= limit) return 1.0f;
    return clamp01(limit / measured);
}
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

    float paddock_scale = 1.0f;
    if (in.paddock_active) {
        const bool temperatures_valid =
            in.controller_temp_left_c >= params.telemetry_temperature_valid_min_c &&
            in.controller_temp_right_c >= params.telemetry_temperature_valid_min_c &&
            in.motor_temp_left_c >= params.telemetry_temperature_valid_min_c &&
            in.motor_temp_right_c >= params.telemetry_temperature_valid_min_c;
        const bool pack_valid =
            !params.paddock_require_pack_data || in.pack_data_valid;
        if (!temperatures_valid || !pack_valid) {
            out.left_a = 0.0f;
            out.right_a = 0.0f;
            out.paddock_limited = true;
            out.paddock_sensor_blocked = true;
            return out;
        }
        if (in.paddock_timed_out) {
            out.left_a = 0.0f;
            out.right_a = 0.0f;
            out.paddock_limited = true;
            out.paddock_timed_out = true;
            return out;
        }
        const float requested_peak = std::fmax(positive(out.left_a),
                                               positive(out.right_a));
        bool phase_current_clamped = false;
        if (out.left_a > params.paddock_current_per_motor_a) {
            out.left_a = params.paddock_current_per_motor_a;
            phase_current_clamped = true;
        }
        if (out.right_a > params.paddock_current_per_motor_a) {
            out.right_a = params.paddock_current_per_motor_a;
            phase_current_clamped = true;
        }
        if (phase_current_clamped) {
            const float limited_peak = std::fmax(positive(out.left_a),
                                                 positive(out.right_a));
            if (requested_peak > 0.0f)
                paddock_scale = limited_peak / requested_peak;
            out.paddock_current_limited = true;
        }

        float speed_scale = 1.0f;
        if (in.paddock_speed_mps >= params.paddock_speed_limit_mps) {
            speed_scale = 0.0f;
        } else if (in.paddock_speed_mps >
                   params.paddock_speed_taper_start_mps) {
            const float band = params.paddock_speed_limit_mps -
                               params.paddock_speed_taper_start_mps;
            speed_scale = band > 0.0f
                ? clamp01((params.paddock_speed_limit_mps -
                           in.paddock_speed_mps) / band)
                : 0.0f;
        }
        paddock_scale *= speed_scale;
        scale_positive(out.left_a, speed_scale);
        scale_positive(out.right_a, speed_scale);

        const float controller_bus_current_sum =
            std::fabs(in.bus_current_left_a) +
            std::fabs(in.bus_current_right_a);
        const float controller_current_scale = positive_limit_scale(
            controller_bus_current_sum,
            params.paddock_controller_bus_current_limit_a);
        const float pack_current_scale = positive_limit_scale(
            std::fabs(in.pack_current_a),
            params.paddock_pack_current_limit_a);
        const float current_scale =
            controller_current_scale < pack_current_scale
                ? controller_current_scale : pack_current_scale;
        if (current_scale < 1.0f) {
            scale_positive(out.left_a, current_scale);
            scale_positive(out.right_a, current_scale);
            paddock_scale *= current_scale;
            out.paddock_current_limited = true;
        }
        out.paddock_limited = true;
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

    float effective_power_limit_w = params.power_soft_limit_w;
    if (in.paddock_active && params.paddock_power_soft_limit_w > 0.0f &&
        (effective_power_limit_w <= 0.0f ||
         params.paddock_power_soft_limit_w < effective_power_limit_w)) {
        effective_power_limit_w = params.paddock_power_soft_limit_w;
    }

    float power_scale = 1.0f;
    if (effective_power_limit_w > 0.0f &&
        governing_power > effective_power_limit_w) {
        power_scale = clamp01(effective_power_limit_w / governing_power);
        scale_positive(out.left_a, power_scale);
        scale_positive(out.right_a, power_scale);
        out.power_limited = true;
    }
    out.applied_scale = thermal_scale * paddock_scale * power_scale;
    return out;
}
