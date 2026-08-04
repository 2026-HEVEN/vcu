// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/longitudinal.h"

namespace {
constexpr float DRIVE_MAX_A_NORMAL = 30.0f;
constexpr float DRIVE_MAX_A_EFF = 20.0f;

// HPM05KW/EZkontrol open-loop regen proposal. The 60 A phase-current result
// has produced repeatable braking torque, but the RPM-shaped curve and the
// battery-current model still require vehicle/BMS validation.
// Total current demand before torque-vectoring allocation. With the current
// 50:50 allocator, 60 A total becomes 30 A per controller.
constexpr float REGEN_MAX_A = 60.0f;
constexpr float REGEN_CUTOUT_RPM = 80.0f;
constexpr float REGEN_FULL_RPM = 500.0f;
constexpr float REGEN_POWER_LIMIT_RPM = 2165.0f;
constexpr float REGEN_POWER_LIMIT_K = 129900.0f; // A*RPM, 30 A battery target

constexpr float SOC_TAPER_START = 0.90f;
constexpr float SOC_TAPER_END = 0.95f;
constexpr float BRAKE_DEADZONE = 5.0f;
constexpr float REGEN_RISE_A_PER_S = 100.0f; // 10 A / 100 ms
constexpr float REGEN_FALL_A_PER_S = 200.0f; // 20 A / 100 ms

float clampf(float value, float lo, float hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}

float move_towards(float current, float target, float max_delta) {
    if (current < target) return current + max_delta < target ? current + max_delta : target;
    if (current > target) return current - max_delta > target ? current - max_delta : target;
    return target;
}

float rpm_regen_limit(float motor_rpm) {
    const float rpm = motor_rpm < 0.0f ? -motor_rpm : motor_rpm;
    if (rpm <= REGEN_CUTOUT_RPM) return 0.0f;

    if (rpm < REGEN_FULL_RPM) {
        const float x = (rpm - REGEN_CUTOUT_RPM) /
                        (REGEN_FULL_RPM - REGEN_CUTOUT_RPM);
        const float smoothstep = x * x * (3.0f - 2.0f * x);
        return REGEN_MAX_A * smoothstep;
    }

    if (rpm <= REGEN_POWER_LIMIT_RPM) return REGEN_MAX_A;

    const float power_limited = REGEN_POWER_LIMIT_K / rpm;
    return power_limited < REGEN_MAX_A ? power_limited : REGEN_MAX_A;
}

float soc_regen_multiplier(float pack_soc) {
    if (pack_soc >= SOC_TAPER_END) return 0.0f;
    if (pack_soc <= SOC_TAPER_START) return 1.0f;
    return 1.0f - ((pack_soc - SOC_TAPER_START) /
                   (SOC_TAPER_END - SOC_TAPER_START));
}
} // namespace

float longitudinal_compute(const LongInput &in,
                           LongitudinalState &state,
                           float dt_s) {
    const float throttle = clampf(in.throttle_pct, 0.0f, 100.0f);
    const float brake = clampf(in.brake_pct, 0.0f, 100.0f);
    const float dt = dt_s > 0.0f ? dt_s : 0.0f;
    const bool braking = brake > BRAKE_DEADZONE;

    const float drive_max = in.mode == DriveMode::Efficiency
                                ? DRIVE_MAX_A_EFF
                                : DRIVE_MAX_A_NORMAL;
    const float drive = braking ? 0.0f : throttle * drive_max / 100.0f;

    float regen_target = 0.0f;
    if (braking) {
        regen_target = rpm_regen_limit(in.motor_rpm) * brake / 100.0f;
        regen_target *= soc_regen_multiplier(clampf(in.pack_soc, 0.0f, 1.0f));
    }

    // High SOC is a protection condition: do not delay the cutoff with a ramp.
    if (in.pack_soc >= SOC_TAPER_END) {
        state.regen_current_a = 0.0f;
    } else {
        const float rate = regen_target > state.regen_current_a
                               ? REGEN_RISE_A_PER_S
                               : REGEN_FALL_A_PER_S;
        state.regen_current_a = move_towards(
            state.regen_current_a, regen_target, rate * dt);
    }

    return drive - state.regen_current_a;
}
