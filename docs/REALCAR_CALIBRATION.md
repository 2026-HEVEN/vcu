# 실차 캘리브레이션 기준

이 문서는 트랙에서 실제로 조정할 값만 다룬다. 펌웨어 통신 주기, CAN 큐,
타임아웃, 고정 제원과 벤치시험 설정은 `src/modules/fixed_config.h`에 있다.

## 설정 파일 역할

| 파일 | 변경 시점 | 대표 항목 |
|---|---|---|
| `src/modules/realcar_calibration.h` | 센서 보정·실차 튜닝·기능 검증 상태가 바뀔 때 | 스로틀/기어 범위, 전류 상한, 출발 램프, 열/패독 한계, 차체·조향·WSS 필터 |
| `src/modules/fixed_config.h` | 하드웨어·프로토콜·스케줄러·시험 절차가 바뀔 때만 | 24 PPR, 감속비, Kt, 제어/CAN 주기, stale/재접속, RX 큐, 벤치 펄스 |
| `src/modules/tv/tv_config.h` | TV 실차 튜닝 때 | 게인, 마찰계수, TV 최소속도, Mz 한계 |
| `src/core/board_pins.h` | PCB/하네스가 바뀔 때 | GPIO |

알고리즘 `.cpp`에 차량 숫자를 새로 하드코딩하지 않는다.

## 현재 핵심 상태

- 일반 주행 모델 기반 전력 제한: `ENABLE_DRIVE_POWER_LIMIT = false`.
  공식 Energy Meter를 장착해 시간 정렬·스케일·지연을 검증하기 전에는 제어에
  사용하지 않는다. `DRIVE_POWER_SOFT_LIMIT_W = 8000`은 다음 시험값일 뿐이다.
- 회생: `REGEN_HARDWARE_VALIDATED = false`. 로직은 존재하지만 음의 상전류를
  실차에 내보내지 않는다.
- 브레이크 입력: `BRAKE_SENSOR_INSTALLED = false`.
- 최대 구동 명령: 모터당 500 A. 모터 연속정격이나 배터리 전류 한계가 아니라
  짧은 시험용 VCU 명령 상한이다.
- 출발 상승 램프: 0.5 s. 감소·페달 해제·고장 차단은 즉시다.
- 재연결 램프: 1.0 s이며 `fixed_config.h`에 있다. 통신 복구 순간의 토크 스텝을
  막는 별도 무결성 기능이라 출발 램프와 합치지 않는다.
- WSS: 상승엣지 기준 24 PPR, 유효 구름반경 0.2387 m.
- 기어 ADC: N `[0,500)`, R `[500,2500)`, D `[2500,4095]`.

Paddock 모드는 별도 8 kW/버스전류/팩전류/속도별 상전류 제한을 아직 가진다.
일반 주행 제한을 false로 바꿔도 Paddock 제한은 꺼지지 않는다.

## 실차에서 조정하는 값

### 매 시험 전에 확인

- `THROTTLE_RAW_MIN/MAX`, `THROTTLE_SIGNAL_VALID_MIN_ADC`
- `GEAR_REVERSE_ADC`, `GEAR_DRIVE_ADC`
- `DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A`
- `BRAKE_SENSOR_INSTALLED`, `REGEN_HARDWARE_VALIDATED`
- 실제 사용할 경우 Paddock 전류·전력·팩전류 한계

### 계측 후 조정

- `DRIVE_CURRENT_RISE_TIME_S`: 0 / 0.25 / 0.5 s A/B 시험
- `WHEEL_SPEED_ROLLING_RADIUS_M`, `WSS_FILTER_TIME_CONSTANT_S`
- 조향 center/range/invert와 최대 실제 조향각
- 차량 질량·윤거·축거·CG·하중배분
- 열 디레이팅 시작/차단점
- Energy Meter 검증 뒤 전력 제한 enable/limit/제어식

## 0.5초 출발 램프 판정

이 램프는 하드웨어 과전류 보호가 아니라 운전감과 DC 전력 과도응답을 다듬는
기능이다. EZkontrol은 자체 상전류/과전류/과열 보호를 가지므로 부품 파손의 최종
보호를 VCU 램프에 의존하면 안 된다. 다만 컨트롤러 앱의 실제 acceleration/ramp
설정이 좌우 동일한지 아직 증명되지 않았고, VCU 램프는 두 모터에 동일한 명령
기울기를 보장한다. 따라서 현재 0.5 s를 유지하고 다음 로그를 비교한다.

1. 같은 SOC·노면·전류상한에서 0 / 0.25 / 0.5 s 시험
2. 스로틀, VCU 명령 L/R, 실제 phase current L/R, bus 전력, 가속도 기록
3. 컨트롤러 내부 램프가 이미 더 느리면 VCU 램프 제거
4. VCU 램프가 좌우 전류 상승차나 10 kW 오버슈트를 유의미하게 줄이면 유지

## 필수 검증 순서

1. 잭업: 스로틀/기어 범위, 양쪽 handshake·freshness, 전진 RPM 극성
   `L>0/R<0`, 네 WSS 24 pulse/rev, IMU·조향 validity 확인.
2. 저속 직선: 구름반경, WSS dropout, 좌우 명령/실제 전류 비교.
3. 출발 램프 A/B 시험.
4. Energy Meter 장착 후 VCU·컨트롤러·EM 로그 시간정렬.
5. 그 뒤에만 전력 제한을 활성화하고, 마지막에 TV/회생을 각각 별도 활성화.

## 빌드 확인

```powershell
platformio test -e native
platformio run -e esp32dev
git diff --check
```

로컬 native 테스트에는 PATH에서 `gcc`와 `g++`가 보여야 한다. ESP32 빌드 성공과
native 테스트 성공은 별개다.
