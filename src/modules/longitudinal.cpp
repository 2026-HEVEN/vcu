// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/longitudinal.h"
#include "modules/realcar_calibration.h"
#include <cmath>

bool regen_voltage_ok(float em_v, bool em_fresh, float pack_v, bool pack_valid) {
    const auto below = [](float v) {
        return std::isfinite(v) && v > 0.0f &&
            v < realcar_cal::bringup::REGEN_VOLTAGE_CUTOFF_V;
    };
    return (em_fresh || pack_valid) &&
        (!em_fresh || below(em_v)) && (!pack_valid || below(pack_v));
}

bool regen_release_update(float throttle_pct, bool valid, RegenReleaseState &state) {
    constexpr unsigned RELEASE_SAMPLES = 10;
    if (!valid || !std::isfinite(throttle_pct) || throttle_pct != 0.0f) {
        state.zero_samples = 0;
        return false;
    }
    if (state.zero_samples < RELEASE_SAMPLES) ++state.zero_samples;
    return state.zero_samples >= RELEASE_SAMPLES;
}

float longitudinal_compute(const LongInput &in) {
    // This module outputs the SUM of the two motor phase-current demands.
    // TV OFF divides it 50:50, so 2 * per-motor limit gives each controller
    // exactly the configured per-motor ceiling at full throttle.
    constexpr float DRIVE_MAX_A_NORMAL =
        2.0f * realcar_cal::bringup::DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A;
    constexpr float DRIVE_MAX_A_EFF =
        2.0f * realcar_cal::bringup::DRIVE_PHASE_CURRENT_EFF_PER_MOTOR_A;
    
    constexpr float SOC_TAPER_START = 0.90f; // 회생제동 감소 시작
    constexpr float SOC_TAPER_END = 0.95f;   // 회생제동 완전 차단

    float drive_max_a = DRIVE_MAX_A_NORMAL;
    float regen_max_a = 0.0f;
    const unsigned level = in.regen_level > 3 ? 0 :
        (in.regen_level == 0 ? 0 : (in.four_stage ? in.regen_level : 3));
    switch (level) {
    case 1: regen_max_a = realcar_cal::bringup::REGEN_LEVEL1_TOTAL_CURRENT_A; break;
    case 2: regen_max_a = realcar_cal::bringup::REGEN_LEVEL2_TOTAL_CURRENT_A; break;
    case 3: regen_max_a = realcar_cal::bringup::REGEN_LEVEL3_TOTAL_CURRENT_A; break;
    default: break; // OFF or malformed level must never request regen
    }

    // 1. 주행 모드에 따른 전략 (Efficiency 모드)
    if (in.mode == DriveMode::Efficiency) {
        drive_max_a = DRIVE_MAX_A_EFF; // 효율 모드: 가속력 제한
        regen_max_a = std::fmin(regen_max_a,
            realcar_cal::bringup::REGEN_EFF_TOTAL_CURRENT_CAP_A);
    }

    // 2. 배터리 과충전 방지 로직 (선형 보간법 적용)
    float regen_multiplier = 1.0f; 

    if (in.pack_soc >= SOC_TAPER_END) {
        regen_multiplier = 0.0f; 
    } else if (in.pack_soc > SOC_TAPER_START) {
        regen_multiplier = 1.0f - ((in.pack_soc - SOC_TAPER_START) / (SOC_TAPER_END - SOC_TAPER_START));
    }
    regen_max_a *= regen_multiplier;

    // Positive throttle always requests propulsion. Once the pedal has been
    // released long enough, the caller enables automatic coast regeneration;
    // brake-trigger mode additionally requires an active installed brake.
    if (!std::isfinite(in.throttle_pct) || in.throttle_pct < 0.0f) return 0.0f;
    if (in.throttle_pct > 0.0f)
        return std::fmin(in.throttle_pct, 100.0f) / 100.0f * drive_max_a;
    if (!in.regen_auto_enabled || (!in.one_pedal && !in.brake_active) ||
        !std::isfinite(regen_max_a) || regen_max_a < 0.0f ||
        !std::isfinite(in.pack_soc) ||
        in.pack_soc < 0.0f || in.pack_soc > 1.0f) return 0.0f;
    return -regen_max_a;
}
