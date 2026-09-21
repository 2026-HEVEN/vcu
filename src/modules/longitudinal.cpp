// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/longitudinal.h"
#include "modules/realcar_calibration.h"
#include <cmath>

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
    constexpr float REGEN_MAX_A_NORMAL = 20.0f;
    constexpr float DRIVE_MAX_A_EFF =
        2.0f * realcar_cal::bringup::DRIVE_PHASE_CURRENT_EFF_PER_MOTOR_A;
    constexpr float REGEN_MAX_A_EFF = 30.0f;
    
    constexpr float SOC_TAPER_START = 0.90f; // 회생제동 감소 시작
    constexpr float SOC_TAPER_END = 0.95f;   // 회생제동 완전 차단
    constexpr float BRAKE_DEADZONE = 5.0f;   // 브레이크 노이즈 무시 구간

    float drive_max_a = DRIVE_MAX_A_NORMAL;
    float regen_max_a = REGEN_MAX_A_NORMAL;

    // 1. 주행 모드에 따른 전략 (Efficiency 모드)
    if (in.mode == DriveMode::Efficiency) {
        drive_max_a = DRIVE_MAX_A_EFF; // 효율 모드: 가속력 제한
        regen_max_a = REGEN_MAX_A_EFF; // 효율 모드: 회생제동 극대화
    }

    // 2. 배터리 과충전 방지 로직 (선형 보간법 적용)
    float regen_multiplier = 1.0f; 

    if (in.pack_soc >= SOC_TAPER_END) {
        regen_multiplier = 0.0f; 
    } else if (in.pack_soc > SOC_TAPER_START) {
        regen_multiplier = 1.0f - ((in.pack_soc - SOC_TAPER_START) / (SOC_TAPER_END - SOC_TAPER_START));
    }
    regen_max_a *= regen_multiplier;

    // Explicit policy: positive throttle wins even with the brake pressed.
    // Do not subtract regen from propulsion or apply a brake-throttle override.
    if (!std::isfinite(in.throttle_pct) || in.throttle_pct < 0.0f) return 0.0f;
    if (in.throttle_pct > 0.0f)
        return std::fmin(in.throttle_pct, 100.0f) / 100.0f * drive_max_a;
    if (!in.regen_auto_enabled || !std::isfinite(in.brake_pct) ||
        in.brake_pct <= BRAKE_DEADZONE || !std::isfinite(in.pack_soc) ||
        in.pack_soc < 0.0f || in.pack_soc > 1.0f) return 0.0f;
    return -std::fmin(in.brake_pct, 100.0f) / 100.0f * regen_max_a;
}
