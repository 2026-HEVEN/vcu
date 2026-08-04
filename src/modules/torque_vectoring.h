#pragma once
#include "types.h"
#include "modules/tv/yaw_control.h"   // TVYawState (제어기 이력) 재노출
// ============================================================
//  [ORCHESTRATOR] 토크벡터링 통합 이음새 — 수정 불필요(코어 담당 영역).
//  실제 로직은 src/modules/tv/ 의 5개 stage에 있습니다. 팀원은 거기만 채우세요.
// ============================================================

struct TVInput {
    float total_torque;    // longitudinal 출력 (부호 있는 총 토크)
    float yaw_rate;        // IMU 실측 yaw rate [deg/s]
    float steering_angle;  // Unit(-1..+1)
    float vehicle_speed;   // 추정 차속 [m/s] (vehicle_speed 모듈 출력)
    float ax;              // 종가속도 [g] (드라이버 출력; TV 진입 시 m/s²로 환산 §2.2)
    float ay;              // 횡가속도 [g]
    float dt;              // tick 간격 [s]
};

struct TVOutput {
    // 컨트롤러로 나가는 실제 명령
    Percent torque_L;
    Percent torque_R;
    // 관측용 중간신호 (app_wiring이 VehicleState로 복사 → debug/Cluster에서 보임)
    float desired_yaw_rate;
    float yaw_moment;
    float fz_L, fz_R;
    float max_torque_L, max_torque_R;
};

// 5개 stage를 순서대로 조립한다. s는 yaw 제어기 이력(코어가 static으로 보유).
TVOutput tv_compute(const TVInput &in, TVYawState &s);
