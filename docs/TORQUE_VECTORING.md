# 토크벡터링 개발 가이드 (팀원용)

토크벡터링(TV)은 이 펌웨어에서 **개발할 게 가장 많은 부분**입니다. 그래서 한 함수에 몰아넣지 않고
**5개 stage(순수 함수)** 로 나눴습니다. 각 stage는 입력→출력이 명확해서 **한 명씩 맡아 병렬로**
개발하고, 하드웨어 없이 노트북에서 테스트할 수 있습니다.

## 파이프라인 한눈에

```
[1 reference]  조향각 + 차속            → 목표 yaw rate        (바이시클 모델)
[2 yaw]        목표 yaw − IMU 실측 yaw   → 요 모멘트 Mz         (PID, 이력 필요)
[3 load]       종/횡 가속도             → 바퀴별 수직하중 Fz    (하중이동, 모델 기반)
[4 traction]   Fz × μ (마찰원)          → 바퀴별 최대 종토크    (휠스핀 방지)
[5 allocation] 총토크 + Mz, 상한 제약    → torque_L / torque_R  (최종 배분)
```

`src/modules/torque_vectoring.cpp`(오케스트레이터)가 이 5개를 순서대로 호출합니다.
**오케스트레이터와 각 `.h`(입출력 계약)는 건드리지 말고**, 아래 `.cpp` 본문만 채우세요.

## 파일 ↔ 담당 ↔ 테스트

| Stage | 채울 파일 | 함수 | 테스트 |
|-------|-----------|------|--------|
| 1 레퍼런스 | `src/modules/tv/reference.cpp` | `tv_reference_compute` | `test/test_tv_reference/` |
| 2 yaw 제어 | `src/modules/tv/yaw_control.cpp` | `tv_yaw_compute` | `test/test_tv_yaw/` |
| 3 하중 추정 | `src/modules/tv/load.cpp` | `tv_load_compute` | `test/test_tv_load/` |
| 4 트랙션 | `src/modules/tv/traction.cpp` | `tv_traction_compute` | `test/test_tv_traction/` |
| 5 배분 | `src/modules/tv/allocation.cpp` | `tv_alloc_compute` | `test/test_tv_alloc/` |

> 각 `.cpp` 맨 위 `담당: ______` 칸에 이름을 적고, 그 stage의 owner가 되세요.

## 지금 상태 = 안전한 stub

5개 다 **안전한 pass-through** 로 채워져 있습니다: 레퍼런스=0, Mz=0, 하중=좌우 동일,
트랙션=무제한, 배분=50:50. → **조립 결과가 지금 차 거동과 100% 동일**하므로, 여러분이
한 stage씩 채워도 나머지는 안전하게 통과만 시킵니다. 하나씩 점진적으로 켜세요.

## 튜닝 상수는 한 곳에서

차량 제원·μ·PID 게인 등은 전부 **`src/modules/tv/tv_config.h`의 `TVParams`** 에 있습니다.
각 `.cpp`에 상수를 흩뿌리지 말고 여기서만 바꾸세요. (지금 값은 전부 임시 placeholder — 실측 필요)

## 중간신호는 눈에 보입니다

각 stage의 중간 출력(목표 yaw, Mz, 바퀴별 Fz, 최대토크)은 `VehicleState`로 복사되어
**시리얼 debug monitor(5Hz)와 Cluster에서 실시간 관측**됩니다. yaw 제어기 튜닝 시 필수예요.
`pio device monitor` 로 `TV: yaw*=… Mz=… Fz(L/R)=… Tmax(L/R)=…` 줄을 보세요.

## 작업 사이클

1. 자기 stage의 `.cpp` 본문 작성 (`.h`는 읽기만)
2. 같은 이름의 `test/` 에 불변식 테스트 추가 (파일 안 `TODO` 목록 참고)
3. `pio test -e native -f test_tv_<stage>` 로 통과 확인
4. `pio run -e esp32dev` 빌드 그린 확인 → PR

## 규칙 (코어와 동일)

- stage `.cpp` 에서 `state.h` / `<Arduino.h>` **include 금지** (native 빌드가 깨짐 = 방어장치)
- 이력(적분/필터)은 전역변수 대신 **상태 struct 인자**로 (yaw 제어기의 `TVYawState`)
- 입출력은 도메인 타입(`Percent`/`Unit`/`Rpm`)으로 — 범위 자동 보장

## 단계별 부호 규약 (먼저 합의할 것)

reference/yaw/load/allocation이 **같은 좌표계·부호 규약**을 써야 합니다(예: 좌회전 = yaw +).
개발 시작 전에 TV팀이 이 규약 하나를 못 박고 각 `.cpp` 주석에 적어두세요.
