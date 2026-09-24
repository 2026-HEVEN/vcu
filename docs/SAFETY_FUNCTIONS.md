# VCU 안전·보호 기능 전수표

> 2026-09-24 구현 업데이트: [M2 주행 복구·진단 정리](M2_DRIVE_CLEANUP_20260924.md)를 우선 참조한다. 현재 fault의 양쪽 차단은 유지하되, fault 해소 후 `FAULT_REARM`으로 명시적 재무장이 가능해졌다. 스로틀 하한은 200, 구동 상한은 400A/모터이며 아래 9월21일 표의 500A/영구 latch 표현은 과거 상태다.

기준: `fix/m2-command-snapshot`의 2026-09-21 작업본. 여기서 “보호”는 차량을
정지시키는 기능뿐 아니라 잘못된 명령 생성, 통신 단절, 센서 오류 및 시험 명령의
오사용을 막는 게이트를 포함한다.

## 출처 구분

- **GM 공식**: Golden Motor EZkontrol 사용자 설명서/제품 페이지. 컨트롤러 자체에
  phase overcurrent, overload, over/undervoltage, controller/motor overtemperature,
  stall, phase/sensor, throttle, CAN 통신 오류 보호가 명시돼 있다.
- **CAN 계약**: `docs/CAN_PROTOCOL.md`, 제조사 CAN 문서와 기존 reference driver.
- **실차 로그**: 2026-09-05 및 2026-09-21 로그에서 확인한 좌우 RPM 극성,
  재핸드셰이크 및 전류/온도 동작.
- **팀 정책**: 운전자 요구와 양쪽 모터 무결성을 위해 VCU에서 정한 조건.
- **규정**: Energy Meter 출력 기준 10 kW.
- **미확정**: BMS 세부 차단값과 EZkontrol 앱의 실제 acceleration/ramp 설정은
  설정 덤프가 없어 아직 출처를 확정할 수 없다.

## 반드시 유지할 VCU 고유 기능

| 기능 | 동작 | 코드 위치 | 출처 | 판단 |
|---|---|---|---|---|
| 스로틀 단선 하한 | raw ADC가 설정 하한 미만이면 invalid·0% | `src/core/app_wiring.cpp` `throttle_update`, `src/modules/throttle.cpp` | 하네스/실측 | 유지. 컨트롤러는 VCU가 만든 CAN 명령만 보므로 원 페달 ADC 오류를 대신 알 수 없다. |
| 기동 시 페달 해제 | 최초 기동은 valid throttle가 1% 이하로 300 ms인 뒤 Drive 허용. 주행 중 Halt에서는 스로틀·핸드셰이크·heartbeat 복구 시 페달 유지 상태로 자동 재무장 | `src/core/safety.cpp`, `src/logic/safety_logic.cpp` | 팀 정책/2026-09-24 실차 로그 | 최초 기동 인터록 유지. 복귀 시 0.5초 전류 상승 제한 재적용. |
| D/R 전환 인터록 | 정지(좌우 50 rpm 이하)+페달 해제 300 ms 뒤 방향 arm | `src/core/app_wiring.cpp`, `src/modules/direction_interlock.cpp` | 실차/팀 정책 | 유지. 컨트롤러의 개별 방향 보호로 대체 불가. |
| 양쪽 컨트롤러 준비 | L/R 모두 handshake 완료 전 양쪽 토크 금지 | `src/core/can_bus.cpp`, `src/logic/safety_logic.cpp` | CAN 계약/듀얼모터 정책 | 유지. 한쪽만 구동되는 yaw 방지. |
| 좌우 명령 스냅샷 | 한 tick의 전류·기어·허용상태를 임계구역에서 한 번에 게시/복사 | `src/core/app_wiring.cpp`, `src/core/can_bus.cpp`, `src/logic/safety_logic.cpp` | 동시성 설계 | 유지. 두 프레임 동시 도착이 아니라 동일한 계산 시점 보장이 목적. |
| 스냅샷 freshness | 게시 후 100 ms가 지난 명령은 양쪽 0/run=false | `src/logic/safety_logic.cpp`, `src/core/can_bus.cpp` | 팀 정책 | 유지. 멈춘 제어 코어의 마지막 명령 재사용 방지. |
| 스케줄러 deadman | 명령 갱신이 200 ms 넘으면 즉시 Halt, 회복 시 자동 재무장 | `src/core/can_bus.cpp`, `src/core/safety.cpp` | 팀 정책 | 외부 명령 freshness가 아니라 scheduler watchdog. |
| feedback freshness | 각 컨트롤러 Part I+II가 250 ms 이내가 아니면 양쪽 명령 차단 | `src/core/can_bus.cpp`, `src/modules/drive_supervisor.cpp` | CAN 계약/실차 장애 | 유지. 컨트롤러 내부 timeout보다 먼저 대칭 차단. |
| TX 실패 대칭 차단 | 연속 3회 queue 실패 시 링크 무효화, 양쪽 토크 차단 | `src/core/can_bus.cpp` | TWAI/팀 정책 | 유지. 한쪽 프레임만 계속 유실되는 경우 대응. |
| bus-off 복구 | handshake 무효화→TWAI recovery/restart→재연결 | `src/core/can_bus.cpp` | ESP-IDF 동작/실차 장애 | 유지. |
| 재핸드셰이크 | 750 ms feedback 손실 또는 새 0x55 probe에서 링크 무효화·재연결 | `src/core/can_bus.cpp` | CAN 계약/실차 로그 | 유지. |
| 재연결 1초 램프 | 통신 복구 후 held throttle를 0→1로 양쪽 동일 적용 | `src/core/can_bus.cpp` | 실차 장애 대응 | 유지. 0.5초 출발 램프와 목적이 다르다. |
| NaN/범위 방어 | 비유한 명령 차단, Amp/Percent/Rpm 도메인 clamp·통계 | `src/logic/safety_logic.cpp`, `include/types.h` | 소프트웨어 무결성 | 유지. |
| TV fail-off/no-add | IMU·차속·최소속도·게인 조건 불충족 시 50:50, 차등전류가 총 요구를 만들지 않음 | `src/modules/tv/gate.cpp`, `src/modules/tv/allocation.cpp` | 제어 안전 설계 | 유지. |
| Cluster 요청 stale | 200 ms 동안 명령이 없으면 TV/회생/Paddock/debug 요청 OFF | `src/core/can_bus.cpp` | 통신 무결성 | 유지. 기본 주행 스로틀은 Cluster 명령에 의존하지 않는다. |
| 회생 다중 게이트 | Cluster ON, D기어, 스로틀 0% 100 ms, BMS valid, 전진극성 L>+50/R<-50, hardware validated 모두 만족. 브레이크 입력은 불필요 | `src/core/app_wiring.cpp`, `src/modules/motor_direction.cpp` | 실차 로그/시험 정책 | 현재 M2 시험 설정에서는 `REGEN_HARDWARE_VALIDATED=true`. |
| 시험 명령 게이트 | throttle released, D, 정지, fresh/fault-free, 제한전류·시간·deadline | `src/core/debug_monitor.cpp`, `src/core/can_bus.cpp`, `src/modules/time_sync_pulse.cpp` | 벤치시험 절차 | 시험 기능을 남길 경우 유지. 장기적으로 production build에서 제외 가능. |

## 컨트롤러/BMS와 겹치는 기능

| VCU 기능 | 현재 코드 | 하위 장치 기능 | 판정 |
|---|---|---|---|
| 모터당 명령 상한 500 A | `longitudinal.cpp`, `tv/allocation.cpp`, `drive_supervisor.cpp` | EZkontrol 앱의 Max phase current와 내부 과전류 보호 | **둘 다 유지.** VCU 값은 의도한 요구 상한, 컨트롤러 값은 실제 하드웨어 최종 상한이다. 두 설정을 동일 의미로 보지 않는다. |
| 컨트롤러 fault latch | `can_bus.cpp`의 Part II `any_fault()` | EZkontrol이 과전류·전압·온도·stall·sensor·CAN fault 생성 | **유지.** 중복 보호가 아니라 컨트롤러 판정을 받아 양쪽을 대칭 정지시키는 상위 조정이다. |
| VCU 열 디레이팅 75→85/100→120 ℃ | `drive_supervisor.cpp` | EZkontrol 자체 controller/motor overtemperature | **검증 후 단순화 후보.** GM 설명서는 controller overtemperature를 70 ℃ 초과로 설명하므로 현재 VCU controller 시작점 75 ℃는 기본 설정이라면 선행 보호가 아니다. 앱의 실제 임계값/센서 스케일을 확인한 뒤 VCU는 logging-only 또는 더 낮은 soft derate 하나만 남긴다. |
| phase feedback 1000 A latch | `can_bus.cpp` | EZkontrol Max phase/overcurrent | **보호로는 제거 후보.** 1000 A는 부품 보호값이 아니며 malformed feedback 진단 역할만 한다. 진단 flag로 바꾸고 영구 Halt와 분리하는 것이 낫다. |
| Paddock controller bus 200 A/BMS 150 A 제한 | `drive_supervisor.cpp` | 컨트롤러 bus limit·BMS overcurrent disconnect | **재설계 후보.** 느린/지연된 BMS 값을 빠른 제어에 쓰면 불안정할 수 있다. 하드 차단 전에 부드럽게 줄이는 목적은 유효하지만 Energy Meter 검증 뒤 하나의 DC 전력 제한으로 통합하는 편이 단순하다. |
| SOC 90→95% 회생 taper | `longitudinal.cpp` | BMS charge overvoltage/overcurrent cutoff | **유지하되 미완성.** BMS 차단은 최후 보호이며 VCU는 애초에 회생을 요청하지 않아야 한다. 향후 SOC 대신 charge-enable/충전전력 한계를 우선 사용한다. |
| 일반 8 kW 추정 제한 | `drive_supervisor.cpp`, `realcar_calibration.h` | BMS/컨트롤러 보호와 목적이 다름; 대회 규정용 | **현재 OFF.** `ENABLE_DRIVE_POWER_LIMIT=false`. Energy Meter 실차 검증 뒤 EM 기반 단일 제한기로 교체하고 기존 Kt/bus/prediction 혼합식은 삭제 후보. |

## 아직 판단 보류인 0.5초 출발 램프

`DRIVE_CURRENT_RISE_TIME_S=0.5`는 상승 요구에만 적용되고 감소/차단은 즉시다.
EZkontrol 내부에 acceleration 설정과 전류루프가 있으므로 중복 가능성이 있지만,
현재 좌우 설정 덤프와 실제 phase-current step response가 없다. 지금 삭제하면 두
컨트롤러 설정 차이를 VCU가 가릴 기회도 사라진다.

판정 시험은 같은 SOC와 전류 상한에서 0/0.25/0.5 s를 비교하고, VCU 명령,
좌우 실제 phase current, bus 전력, 가속도, 휠슬립을 기록한다. 컨트롤러가 어느
경우든 같은 더 느린 기울기를 만들면 VCU 램프를 제거한다. VCU 램프가 좌우 상승차
또는 전력 overshoot를 줄이면 유지한다.

## 보호처럼 보이지만 실제로 없는 것

- `shutdown_ok`는 `src/core/safety.cpp`에서 현재 항상 `true`다. 하네스 v5의 물리
  shutdown chain은 외부 하드와이어이고 VCU가 상태를 읽지 않는다.
- 별도 Start 버튼 입력도 없다. safety FSM의 `start_pressed`에는 “스로틀 해제
  300 ms 완료”가 들어간다.
- 기어 ADC는 0~4095 전 구간을 N/R/D로 분류하므로 단선/short 전용 invalid band가
  없다. 실측 회로가 양 끝 fault를 구분할 수 있을 때만 간단한 invalid band를 추가한다.
- BMS 보호 설정값과 charge-enable 신호는 VCU가 직접 검증하지 못한다. 현재
  Cluster BMS 프레임의 valid/connected/SOC만 사용한다.

## 정리 결론

1. 통신·좌우대칭·기어·스로틀·회생 게이트는 VCU 고유 역할이라 유지한다.
2. 일반 모델 기반 전력 제한은 OFF 상태를 유지하고 Energy Meter 단일 경로로 교체한다.
3. 0.5초 출발 램프는 A/B 로그 전까지 유지, 1초 재연결 램프는 영구 유지한다.
4. 열 디레이팅, 1000 A latch, Paddock 다중 제한은 하위 장치 설정을 확보한 뒤
   logging-only 또는 하나의 상위 제한기로 단순화한다.
5. component test/time-sync는 production 기능에서 분리할 후보지만, 현 단계의
   계측 검증이 끝날 때까지 `fixed_config::bench`에 격리해 유지한다.

## 외부 근거

- Golden Motor, *EZkontrol Users Manual*, System Protection Characteristics:
  overcurrent, overload, over/undervoltage, controller/motor overtemperature,
  stall, phase/sensor, throttle, CAN protection.
- Golden Motor EZkontrol 제품 페이지: programmable current/power levels,
  regenerative braking, motor temperature protection, excessive-current protection.
- 프로젝트 내부 `docs/CAN_PROTOCOL.md`, `docs/M2_COMMAND_SNAPSHOT.md` 및
  2026-09-05/2026-09-21 실차 로그.
