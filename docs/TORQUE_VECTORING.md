# 토크벡터링 통합 설계

현재 구현은 후륜 좌·우 모터의 목표 상전류를 배분하는 5단계 파이프라인이다.
`src/modules/torque_vectoring.cpp`가 각 단계를 순서대로 호출한다.

```text
steering + vehicle speed -> desired yaw rate
desired/measured yaw     -> yaw moment Mz
longitudinal/lateral g   -> rear wheel loads Fz
Fz + friction circle    -> per-motor phase-current limits
total phase current + Mz -> left/right phase-current commands
```

## 단위와 부호 계약

- 좌회전 조향, 좌회전 yaw rate, 반시계 Mz는 양수다.
- `ax > 0`은 전방 가속이며 후축 하중을 증가시킨다.
- `ay > 0`은 좌향 가속이며 우측 바퀴 하중을 증가시킨다.
- IMU 가속도 입력은 현재 드라이버 계약에 맞춰 `g` 단위다.
- yaw rate는 `deg/s`, Mz는 `N·m`, Fz는 `N`이다.
- 최종 출력은 백분율이 아니라 모터 목표 상전류 `Amp`다.
- `Amp` 타입은 HPM05KW 피크 표현을 위해 ±300 A를 표현하지만, 기본 연속
  운전 한도는 `TVParams::motor_current_max_a = 103 A`다.

## 단계별 구현

| Stage | 파일 | 현재 정책 |
|---|---|---|
| Reference | `tv/reference.cpp` | 정상원선회 바이시클 모델, 저속·마찰 yaw 상한 |
| Yaw control | `tv/yaw_control.cpp` | PID, measurement derivative, 연속 deadband, 조건부 적분 및 하드 제한 |
| Load | `tv/load.cpp` | 후축 정적하중, 종하중 이동, 별도 LLTD 기반 횡하중 이동 |
| Traction | `tv/traction.cpp` | 마찰원, `Kt×gear×radius` 전류 변환, 잘못된 입력에서 0 A |
| Allocation | `tv/allocation.cpp` | yaw 우선, 공통 전류 축소, 장비/그립 한도와 구동·회생 부호 보존 |

`Mz`의 대칭 좌우 차등 전류는 다음 식으로 계산한다.

```text
diff_A = Mz_Nm * tire_radius_m
         / (track_m * motor_Kt_Nm_per_A * gear_ratio)

I_left  = base_A - diff_A
I_right = base_A + diff_A
```

한쪽이 한계에 도달하면 요청 yaw 차등을 가능한 만큼 우선 보존하고 공통
가감속 전류를 줄인다. 단, 구동 요구에서 음의 전류가, 회생 요구에서 양의
전류가 생성되지 않도록 최종 출력 범위를 제한한다. 총 요구가 0 A이면 TV가
독자적으로 모터 전류를 만들지 않는다.

## 안전 동작

- `vehicle_speed < 1 m/s` 또는 차속 invalid 시 Mz를 0으로 하고 PID 상태를 초기화한다.
- NaN, 잘못된 `dt`, 0/음수 차량·구동계 파라미터는 0 출력으로 처리한다.
- 음의 수직하중과 마찰원 제곱근의 음수 입력을 차단한다.
- 기본 `kp/ki/kd`는 모두 0이다. 실차 식별과 단계별 시험 전에는 TV가 차등
  전류를 만들지 않는다.

## 아직 실측해야 하는 값

- 운전자 포함 질량, 축거, 윤거, CG 높이
- 정적 후축 하중 배분과 후축 LLTD
- steering `Unit`과 실제 타이어 조향각의 매핑
- IMU yaw, 조향 및 좌·우 모터의 실제 부호
- 노면별 `mu`와 좌우 차등 전류의 허용 변화율
- PID 게인과 센서 stale 판단 시간

`Kt=0.1266 N·m/A`, 감속비 `3.72`, 구름반경 `0.2387 m`는 현재 확인된
값을 사용한다. 차량 파라미터는 `src/modules/tv/tv_config.h` 한 곳에서만
변경한다.

## 검증 명령

```powershell
pio test -e native
pio run -e esp32dev
```

실차 검증은 부호 확인, 잭업 저전류, 직선 저속, 정상원선회, 슬라럼 순서로
진행한다. 각 단계에서 좌·우 명령 전류, phase current, yaw 오차, Fz와 전류
한계가 예상 방향으로 움직이는지 로그로 확인한다.
