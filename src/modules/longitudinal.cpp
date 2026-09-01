// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/longitudinal.h"
#include "modules/realcar_calibration.h"

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

    // 3. 최종 토크(전류) 계산
    float drive = (in.throttle_pct / 100.0f) * drive_max_a;
    float regen = in.regen_auto_enabled
        ? (in.brake_pct / 100.0f) * regen_max_a
        : 0.0f;

    // 4. 안전 로직: Brake Override (양발 운전 급발진 방지)
    // [권장 피드백 반영] 센서 노이즈나 발을 살짝 올려둔 상태(데드존)를 무시하기 위해 5% 초과일 때만 구동 차단
    if (in.brake_pct > BRAKE_DEADZONE) {
        drive = 0.0f; 
    }

    // 5. 최종 반환: 구동 전류(+)와 회생제동 전류(-)의 합
    return drive - regen;
}
