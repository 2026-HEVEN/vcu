# 리프트 상태 개별 부품 시험

이 문서는 `bringup/component-test` 브랜치 전용이다. 정상 `dev`와 분리된 상태에서
좌·우 모터, CAN 피드백, 휠스피드 센서, MTi-320 IMU, 기어 ADC를 확인한다.

## 시험 프로파일

- 정상 스로틀 명령 상한: 모터당 150 A
- Efficiency 명령 상한: 모터당 100 A
- 개별 모터 펄스 명령 상한: 모터당 150 A, 100~3000 ms
- 회생제동: 검증 플래그가 꺼져 있어 비활성
- 기어: 실제 구동 권한은 D로 고정하고, 스위치 판정값과 raw ADC만 관측
- TV PID: 0으로 유지
- 개별 모터 펄스는 선택한 컨트롤러만 RUNNING, 반대쪽은 HALTED/0 A

펄스 명령은 스로틀과 브레이크가 놓여 있고, 선택한 컨트롤러가 핸드셰이크 완료,
Part I/II 피드백 fresh, fault 없음, speed mode 아님, 시작 RPM 절댓값 50 이하일 때만
수락한다. 실행 중에도 50 ms마다 deadman, safety Halt, 피드백, fault, 온도,
상전류를 다시 검사한다. 하나라도 실패하거나 시간이 끝나면 0 A/HALTED로 돌아간다.
시험 종료나 중단 뒤에는 페달이 300 ms 동안 놓인 것이 확인될 때까지 정상 스로틀
경로도 재활성화하지 않는다.
물리 비상정지 및 shutdown chain은 시험자가 직접 확보한다.

## 시리얼 명령

115200 bps, newline으로 다음 형식을 보낸다.

```text
MOTOR_L <phase-current-A> <duration-ms>
MOTOR_R <phase-current-A> <duration-ms>
MOTOR_BOTH <phase-current-A-per-motor> <duration-ms>
```

첫 시험은 다음처럼 10 A, 300 ms부터 시작한다.

```text
MOTOR_L 10 300
MOTOR_R 10 300
MOTOR_BOTH 10 300
```

수락 시 `[MOTOR_TEST] accepted`, 거절 시 원인이 출력된다. 별도 소프트웨어 즉시
정지 명령은 두지 않았다. 모든 명령은 최대 3초 안에 자동 종료되며 실제 즉시 정지는
차량의 물리 스위치를 사용한다.

## 로그 판독

- `CAN HS=L/R`: 각 컨트롤러 핸드셰이크
- `fresh=L/R`, `age1`, `age2`: Part I/II 수신 여부와 마지막 수신 이후 ms
- `err`: 컨트롤러 error1/2/3
- `twai`, `txFail`, `rxMiss`, `busErr`, `arbLost`: ESP32 CAN 드라이버 상태/누적 오류
- `MCU Ibus/Iph/rpm`: 좌·우 컨트롤러 실측값
- `TEST active/side/cmd/remain`: 현재 펄스 상태
- `gear`: 실제 제어에 사용하는 기어. 이 브랜치에서는 D(2) 고정
- `sensed/raw`: 임시 ADC 구간으로 판정한 기어와 실제 GPIO27 ADC
- `IMU=ok|STALE`: MTi-320 데이터 freshness
- `WSS`, `pulses`: 네 바퀴 RPM과 부팅 이후 누적 상승엣지. 한 바퀴당 48 증가 예상

기어 ADC의 N/R/D 기준값은 아직 임시값이다. 세 위치에서 `raw` 안정 구간을 먼저
기록하고 `realcar_calibration.h`의 기준값과 허용폭을 보정한 뒤에만
`GEAR_SELECTOR_INSTALLED=true`로 바꾼다.

## 권장 순서

1. 구동 전원을 차단한 채 VCU를 켜고 `thrRaw`, `raw`, `IMU`, WSS 누적 펄스를 확인한다.
2. 각 바퀴를 손으로 한 바퀴 돌려 대응 채널 `pulses`가 약 48 증가하는지 확인한다.
3. 기어 스위치를 N/R/D로 옮겨 각 위치의 `raw` 범위와 `sensed`를 기록한다.
4. CAN을 연결하고 좌·우 `HS=1`, `fresh=1`, `err=000000`, 오류 카운터가 증가하지 않는지 확인한다.
5. 바퀴를 확실히 띄우고 물리 비상정지를 잡은 상태에서 좌 10 A/300 ms부터 시험한다.
6. 좌측 로그를 확인한 뒤 우측, 양쪽 순으로 반복한다. 전류는 로그를 확인하며 단계적으로 올린다.
7. 마지막에 스로틀을 아주 천천히 밟아 좌·우 요청/실측 전류와 RPM 방향을 확인한다.

CAN 오류 카운터가 계속 증가하거나 WSS 채널과 실제 바퀴 위치가 다르거나, IMU가
STALE이거나, 놓은 스로틀이 0%가 아니면 전류를 올리지 않는다.
