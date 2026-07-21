// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/longitudinal.h"

float longitudinal_compute(const LongInput &in) {
    float drive_max_a = 30.0f;
    float regen_max_a = 20.0f;

    // 1. 주행 모드에 따른 전략 (Efficiency 모드)
    if (in.mode == DriveMode::Efficiency) {
        drive_max_a = 20.0f; // 효율 모드: 배터리 소모를 줄이기 위해 가속력 제한
        regen_max_a = 30.0f; // 효율 모드: 회생제동을 강하게 걸어 배터리 충전 극대화
    }

    // 2. 배터리 과충전 방지 로직 (선형 보간법 적용)
    // 90% 미만: 회생제동 100% 허용 (multiplier = 1.0)
    // 90% ~ 95%: 회생제동 한계치를 100% -> 0% 로 부드럽게 감소
    // 95% 이상: 회생제동 완전 차단 (multiplier = 0.0)
    float regen_multiplier = 1.0f; 

    if (in.pack_soc >= 0.95f) {
        regen_multiplier = 0.0f; 
    } else if (in.pack_soc > 0.90f) {
        // (현재 SOC - 90%) / (95% - 90%) 비례식을 이용해 1.0에서 스르륵 깎아냄
        regen_multiplier = 1.0f - ((in.pack_soc - 0.90f) / 0.05f);
    }
    regen_max_a *= regen_multiplier;

    // 3. 최종 토크(전류) 계산
    float drive = (in.throttle_pct / 100.0f) * drive_max_a;
    float regen = (in.brake_pct / 100.0f) * regen_max_a;

    // 4. 안전 로직: Brake Override (양발 운전 급발진 방지)
    if (in.brake_pct > 5.0f) {
        drive = 0.0f; // 브레이크가 조금이라도 밟히면 구동(가속)을 0으로 강제 차단
    }

    // 5. 최종 반환: 구동 전류(+)와 회생제동 전류(-)의 합
    return drive - regen;
}