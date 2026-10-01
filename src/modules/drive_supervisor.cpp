#include "modules/drive_supervisor.h"
#include <cmath>

namespace {
float clamp01(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

float positive(float value) { return value > 0.0f ? value : 0.0f; }

void scale_positive(float &value, float scale) {
    if (value > 0.0f) value *= scale;
}

void scale_all(float &value, float scale) { value *= scale; }

float drive_magnitude(float value, bool propulsion_requested) {
    return propulsion_requested ? std::fabs(value) : positive(value);
}

void scale_drive(float &value, float scale, bool propulsion_requested) {
    if (propulsion_requested) scale_all(value, scale);
    else scale_positive(value, scale);
}

float positive_limit_scale(float measured, float limit) {
    if (limit <= 0.0f || measured <= limit) return 1.0f;
    return clamp01(limit / measured);
}

void reset_rise_limit(DriveSupervisorState &state) {
    state.previous_left_a = 0.0f;
    state.previous_right_a = 0.0f;
}

float limit_rising_magnitude(float target, float previous, float max_step,
                             bool &limited) {
    if (target == 0.0f) return 0.0f;
    const float target_magnitude = std::fabs(target);
    const bool same_direction = target * previous > 0.0f;
    const float previous_magnitude = same_direction
        ? std::fabs(previous) : 0.0f;

    // Reductions are immediate. Only an increase in propulsion magnitude is
    // ramped, so pedal release and every downstream protection remain fast.
    if (target_magnitude <= previous_magnitude) return target;

    const float next_magnitude = std::fmin(
        target_magnitude, previous_magnitude + positive(max_step));
    if (next_magnitude < target_magnitude) limited = true;
    return std::copysign(next_magnitude, target);
}
}

DriveSupervisorOutput drive_supervisor_compute(
    const DriveSupervisorInput &in, const DriveSupervisorParams &params,
    DriveSupervisorState &state) {
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
        reset_rise_limit(state);
        out.controller_blocked = true;
        return out;
    }

    // Temperature telemetry is diagnostic only. Controller-reported faults
    // still use the common fault gate above; no VCU temperature thresholds.
    float paddock_scale = 1.0f;
    if (in.paddock_active) {
        // No pack-data gate for propulsion. Regen eligibility is evaluated
        // separately upstream and at the final motor-command gate.
        out.paddock_current_limit_a = positive(params.paddock_current_max_per_motor_a);

        const float requested_peak = std::fmax(
            drive_magnitude(out.left_a, in.propulsion_requested),
            drive_magnitude(out.right_a, in.propulsion_requested));
        bool phase_current_clamped = false;
        if (drive_magnitude(out.left_a, in.propulsion_requested) >
            out.paddock_current_limit_a) {
            out.left_a = in.propulsion_requested
                ? std::copysign(out.paddock_current_limit_a, out.left_a)
                : out.paddock_current_limit_a;
            phase_current_clamped = true;
        }
        if (drive_magnitude(out.right_a, in.propulsion_requested) >
            out.paddock_current_limit_a) {
            out.right_a = in.propulsion_requested
                ? std::copysign(out.paddock_current_limit_a, out.right_a)
                : out.paddock_current_limit_a;
            phase_current_clamped = true;
        }
        if (phase_current_clamped) {
            const float limited_peak = std::fmax(
                drive_magnitude(out.left_a, in.propulsion_requested),
                drive_magnitude(out.right_a, in.propulsion_requested));
            if (requested_peak > 0.0f)
                paddock_scale = limited_peak / requested_peak;
            out.paddock_current_limited = true;
        }

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
            scale_drive(out.left_a, current_scale, in.propulsion_requested);
            scale_drive(out.right_a, current_scale, in.propulsion_requested);
            paddock_scale *= current_scale;
            out.paddock_current_limited = true;
        }
        out.paddock_limited = true;
    }

    // Cap propulsion before the launch slew limiter so the ramp never aims
    // above the ceiling. The budget is shared: sum of both motors' modelled
    // input power <= 2 * rpm_cap_power_per_motor_w (the 10 kW rule is on the
    // pack total). A per-motor cap with a common scale shrank the TV split at
    // full throttle because the outer motor always hit its own cap first.
    // Only propulsion-direction current counts; TV inner-wheel regen and
    // pedal-release regen are not capped.
    float rpm_cap_scale = 1.0f;
    if (in.propulsion_requested && params.rpm_cap_power_per_motor_w > 0.0f) {
        const float sum = out.left_a + out.right_a;
        const auto modelled_power = [&](float current_a, int rpm) {
            if (current_a * sum <= 0.0f) return 0.0f;
            const float w_per_a = params.rpm_cap_w_per_a_offset +
                params.rpm_cap_w_per_a_per_rpm * std::fabs((float)rpm);
            return std::fabs(current_a) * (w_per_a > 0.0f ? w_per_a : 0.0f);
        };
        const float power =
            modelled_power(out.left_a, in.motor_rpm_left) +
            modelled_power(out.right_a, in.motor_rpm_right);
        const float budget = 2.0f * params.rpm_cap_power_per_motor_w;
        if (power > budget) {
            rpm_cap_scale = budget / power;
            scale_all(out.left_a, rpm_cap_scale);
            scale_all(out.right_a, rpm_cap_scale);
            out.rpm_cap_limited = true;
        }
    }

    // Separate from the motoring DC-input fit. Apply only to negative regen
    // commands, never reverse propulsion. Upstream RPM/BMS/fault gates remain.
    float regen_scale = 1.0f;
    constexpr float TWO_PI_OVER_60 = 0.104719755f;
    if (!in.propulsion_requested && params.regen_mechanical_power_per_motor_w > 0.0f) {
        const auto required_scale = [&](float command, int rpm) {
            if (command >= 0.0f) return 1.0f;
            if (!std::isfinite(command) || !std::isfinite(params.motor_kt_nm_per_a) ||
                params.motor_kt_nm_per_a <= 0.0f || rpm == 0) return 0.0f;
            const float mechanical_power = std::fabs(command) *
                params.motor_kt_nm_per_a * std::fabs((float)rpm) * TWO_PI_OVER_60;
            return positive_limit_scale(mechanical_power,
                params.regen_mechanical_power_per_motor_w);
        };
        regen_scale = std::fmin(required_scale(out.left_a, in.motor_rpm_left),
                                required_scale(out.right_a, in.motor_rpm_right));
        if (regen_scale < 1.0f) {
            scale_all(out.left_a, regen_scale);
            scale_all(out.right_a, regen_scale);
            out.power_limited = true;
        }
    }
    const float efficiency = params.drivetrain_efficiency > 0.05f
        ? params.drivetrain_efficiency : 1.0f;
    const auto estimate_input_power = [&](float left_current_a,
                                          float right_current_a) {
        return
            (std::fabs(left_current_a) * params.motor_kt_nm_per_a *
                 std::fabs((float)in.motor_rpm_left) * TWO_PI_OVER_60 +
             std::fabs(right_current_a) * params.motor_kt_nm_per_a *
                 std::fabs((float)in.motor_rpm_right) * TWO_PI_OVER_60) /
            efficiency;
    };

    // Estimate shaft/input power from the phase current that each controller
    // actually reports, not from the final requested-current target.  Using
    // the target here made a 500 A request pre-emptively reduce launch current
    // even while the shared rise limiter and the motors were still well below
    // that current.
    out.estimated_input_power_w = estimate_input_power(
        in.phase_current_left_a, in.phase_current_right_a);

    float effective_power_limit_w = params.power_soft_limit_w;
    if (in.paddock_active && params.paddock_power_soft_limit_w > 0.0f &&
        (effective_power_limit_w <= 0.0f ||
         params.paddock_power_soft_limit_w < effective_power_limit_w)) {
        effective_power_limit_w = params.paddock_power_soft_limit_w;
    }

    float slew_scale = 1.0f;
    if (in.propulsion_requested) {
        if (params.drive_current_rise_time_s > 0.0f) {
            const float desired_peak = std::fmax(std::fabs(out.left_a),
                                                 std::fabs(out.right_a));
            const float rise_rate_a_per_s =
                positive(params.drive_current_max_per_motor_a) /
                params.drive_current_rise_time_s;
            const float max_step = rise_rate_a_per_s * positive(in.control_dt_s);
            bool slew_limited = false;
            out.left_a = limit_rising_magnitude(
                out.left_a, state.previous_left_a, max_step, slew_limited);
            out.right_a = limit_rising_magnitude(
                out.right_a, state.previous_right_a, max_step, slew_limited);
            if (slew_limited) {
                const float limited_peak = std::fmax(std::fabs(out.left_a),
                                                     std::fabs(out.right_a));
                if (desired_peak > 0.0f)
                    slew_scale = limited_peak / desired_peak;
                out.drive_slew_limited = true;
            }
        }
    }

    // Predict the input power of the current command that will be sent this
    // cycle, after the launch slew limiter.  This catches a likely over-limit
    // command before the 20 Hz controller feedback reports the resulting
    // phase/bus current, without treating the unreached 500 A target as real.
    out.predicted_command_power_w = estimate_input_power(out.left_a, out.right_a);

    float governing_power = out.measured_bus_power_w;
    if (out.estimated_input_power_w > governing_power)
        governing_power = out.estimated_input_power_w;
    if (out.predicted_command_power_w > governing_power)
        governing_power = out.predicted_command_power_w;

    float power_scale = 1.0f;
    if (effective_power_limit_w > 0.0f &&
        governing_power > effective_power_limit_w) {
        power_scale = clamp01(effective_power_limit_w / governing_power);
        scale_drive(out.left_a, power_scale, in.propulsion_requested);
        scale_drive(out.right_a, power_scale, in.propulsion_requested);
        out.power_limited = true;
    }

    if (in.propulsion_requested) {
        // Store the final protected command.  If the power limiter reduced it,
        // the next cycle may only rise from this value, preventing oscillatory
        // jumps back toward the unbounded request.
        state.previous_left_a = out.left_a;
        state.previous_right_a = out.right_a;
    } else {
        // A released/blocked propulsion request resets the launch history so
        // the next Normal or Paddock acceleration starts from zero.
        reset_rise_limit(state);
    }

    out.applied_scale =
        paddock_scale * rpm_cap_scale * regen_scale * power_scale * slew_scale;
    return out;
}
