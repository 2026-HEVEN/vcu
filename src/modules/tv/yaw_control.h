#pragma once
#include "modules/tv/tv_config.h"
// [FILL-IN] Stage 2/5 — yaw 제어기. yaw 오차 → 요 모멘트 Mz.

// 제어기 이력(적분·이전오차)은 전역변수가 아니라 이 struct에 담는다.
// app_wiring이 static으로 하나 들고 매 tick 넘겨준다 (imu.h의 ImuFilterState와 동일 패턴).
// 테스트는 이 상태를 직접 주입해 결정론적으로 검증한다.
struct TVYawState {
    float integral         = 0.0f;
    float prev_measured_yaw= 0.0f; // derivative-on-measurement용 (마지막 새 샘플)
    float derivative       = 0.0f; // 필터를 거친 D 입력 [deg/s^2]
    float sample_dt        = 0.0f; // 마지막 새 샘플 이후 경과 시간 [s]
    bool initialized       = false;
};

// desired_yaw     : 목표 yaw rate [deg/s]  (reference stage 출력)
// measured_yaw    : IMU 실측 yaw rate [deg/s]
// dt              : 이 tick의 시간간격 [s]
// measurement_new : measured_yaw가 새 IMU 샘플인가. 같은 샘플을 다시 읽은 tick이면
//                   false → D는 직전 값을 유지하고, 다음 새 샘플에서 실제 샘플
//                   간격으로 기울기를 계산한다 (생략 시 매 tick 새 샘플로 간주).
// 반환             : 요 모멘트 Mz  (allocation이 좌우 토크차로 환산)
float tv_yaw_compute(float desired_yaw, float measured_yaw, float dt,
                     const TVParams &p, TVYawState &s,
                     bool measurement_new = true);
