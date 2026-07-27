#pragma once
#include "types.h"
// [FILL-IN] 4륜 휠속도 → 차속 추정. 어느 바퀴를 쓸지는 이 모듈이 정한다.
//
// 왜 별도 모듈인가:
//   토크벡터링 reference stage가 바이시클 모델에 넣을 값은 "무게중심의 종방향 속도"다.
//   제어기가 특정 바퀴 rpm을 직접 받으면 (a) 어느 바퀴인지 암묵적 가정이 생기고
//   (b) 타이어 반경 같은 차속 추정 지식이 제어 로직에 새어 들어간다.
//   차속 결정은 여기서 한 번만 하고, 결과를 state에 실어 소비자들이 가져간다.

// 휠 인덱스 — 배열 순서 고정. app_wiring의 GPIO 배정과 반드시 일치시킬 것.
enum WheelIdx { WHEEL_FL = 0, WHEEL_FR = 1, WHEEL_RL = 2, WHEEL_RR = 3, WHEEL_COUNT = 4 };

struct VehicleSpeedCalib {
    // 구름반경 = 실측 구름둘레 1.50 m / 2π  (구름직경 477 mm)
    //   ※ 이전 값 0.165 는 노션 WSS 계산기의 "구름직경 330mm"에서 왔는데,
    //     330 mm 는 13.0 인치로 **림 지름**이다. 타이어 포함 구름직경이 아니다.
    float tire_radius_m  = 0.2387f;
    float track_m        = 1.20f;   // 전륜 윤거 (선회 시 좌우 속도차 보정용)
    float max_accel_mps2 = 15.0f;   // 물리적으로 가능한 최대 가감속. 초과 변화는 센서 이상.
};

struct VehicleSpeedInput {
    Rpm   wheel_rpm[WHEEL_COUNT];   // FL, FR, RL, RR
    float yaw_rate;                 // deg/s (전륜 한쪽만 살아있을 때 선회 보정에 사용)
    float dt;                       // s
};

// 이력(직전 추정 차속)은 전역변수가 아니라 이 struct에 담는다 (ImuFilterState와 동일 패턴).
struct VehicleSpeedState {
    float speed_mps  = 0.0f;
    bool  primed     = false;       // 첫 샘플 여부 (첫 tick은 급변 검사를 건너뛴다)
};

struct VehicleSpeedOutput {
    float speed_mps;                // 추정 차속 [m/s]
    bool  valid;                    // false = 전륜 신호를 못 믿음 → TV는 비활성화할 것
};

// 전륜(비구동륜) 기준으로 차속을 추정한다. 구동륜은 토크로 슬립하므로 쓰지 않는다.
VehicleSpeedOutput vehicle_speed_compute(const VehicleSpeedInput &in,
                                         const VehicleSpeedCalib &c,
                                         VehicleSpeedState &s);
