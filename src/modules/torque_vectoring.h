#pragma once
#include "types.h"
#include "modules/tv/yaw_control.h"   // TVYawState (제어기 이력) 재노출
// ============================================================
//  [ORCHESTRATOR] 토크벡터링 통합 이음새 — 수정 불필요(코어 담당 영역).
//  실제 로직은 src/modules/tv/ 의 5개 stage에 있습니다. 팀원은 거기만 채우세요.
// ============================================================

struct TVInput {
    float total_torque;    // longitudinal 출력: 부호 있는 총 상전류 [A]
    float yaw_rate;        // IMU 실측 yaw rate [deg/s]
    float steering_angle;  // Unit(-1..+1)
    float vehicle_speed;   // 추정 차속 [m/s] (vehicle_speed 모듈 출력)
    float ax;              // 종가속도 [g] (IMU driver 출력 계약)
    float ay;              // 횡가속도 [g]
    float dt;              // tick 간격 [s]
    // 대시보드 TC/TV 스위치 (Cluster CAN_ID_CLUSTER_CMD -> state.tv_enable_requested).
    // false: 기존 strict-OFF와 동일하게 좌우 50:50, 차등(Mz)만 0. 총 추진력은 안 끊음.
    // 주의: 여기 기본 멤버 초기화자를 넣지 말 것 — 일부 ESP32 Xtensa GCC 툴체인이
    // 기본 멤버 초기화자가 있으면 이 struct를 더 이상 aggregate로 취급하지 않아,
    // app_wiring.cpp의 `const TVInput tv_in{...}` 리스트 초기화가 컴파일 실패한다
    // (native 툴체인에서는 통과하지만 esp32dev에서만 실패해서 발견하기 어려움).
    // 안전 기본값은 state.h의 tv_enable_requested=false에서 오므로 여기선 필요 없다.
    bool  tv_enable_requested;
    // 이번 yaw_rate가 직전 tick과 같은 IMU 샘플을 다시 읽은 값인가.
    // true면 yaw D항이 실제 샘플 간격으로 기울기를 계산한다. 생략하면(false)
    // 매 tick 새 샘플로 간주한다. 위와 같은 이유로 기본 멤버 초기화자 금지.
    bool  yaw_sample_repeated;
};

struct TVOutput {
    // 컨트롤러로 나가는 실제 명령
    Amp torque_L;          // motor phase-current command [A]
    Amp torque_R;
    // 관측용 중간신호 (app_wiring이 VehicleState로 복사 → debug/Cluster에서 보임)
    float desired_yaw_rate;
    float yaw_moment;
    float fz_L, fz_R;
    float max_torque_L, max_torque_R;
    bool control_active; // actual pipeline gate, independent of nonzero Mz
};

// 오케스트레이터 상태 (코어가 static으로 하나 보유).
// yaw는 TV가 꺼질 때마다 초기화되는 제어기 이력이고, 나머지는 꺼져 있는
// 동안에도 유지해야 하는 게이트 상태다.
struct TVState {
    TVYawState yaw{};
    bool  speed_gate_open = false; // 저속 게이트 (히스테리시스)
    float mz_ramp         = 0.0f;  // 0..1, TV가 켜진 직후 적용 Mz 배율
};

// 5개 stage를 순서대로 조립한다. 차량은 기본 TV_PARAMS를 쓴다.
TVOutput tv_compute(const TVInput &in, TVState &s);
// 같은 파이프라인에 파라미터를 주입한다 (게인을 켠 통합 테스트·튜닝용).
TVOutput tv_compute(const TVInput &in, TVState &s, const TVParams &p);
