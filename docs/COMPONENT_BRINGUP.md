# PCB V3 CAN·모터·WSS 시리얼 벤치 시험

이 문서는 `bench/pcb-v3-can-motor-wss` 브랜치 전용이다. 미완성 하네스에서
PCB V3 VCU와 좌·우 모터컨트롤러의 CAN, 핸드셰이크, 짧은 구동 및 WSS만 확인한다.
정상 `dev` 주행 펌웨어로 사용하지 않는다.

## 시험 프로파일

- 정상 스로틀·기어 구동: 코드에서 강제 비활성
- 시리얼 모터 펄스 명령 상한: 모터당 10 A, 100~300 ms
- 좌·우 컨트롤러가 모두 handshake/fresh 상태여야 모든 펄스 명령 수락
- 회생제동: 검증 플래그가 꺼져 있어 비활성
- 스로틀·기어·브레이크 입력: 시리얼 펄스 시험에서는 사용하지 않음
- TV PID: 0으로 유지
- 개별 모터 펄스는 선택한 컨트롤러만 RUNNING, 반대쪽은 HALTED/0 A
- Time Sync 구동 펄스: 비활성

펄스 명령은 좌·우 모두 핸드셰이크 완료, Part I/II 피드백 fresh, fault 없음,
speed mode 아님, 시작 RPM 절댓값 50 이하일 때만 수락한다. 실행 중에도 50 ms마다
양쪽 피드백·fault·온도·상전류를 다시 검사한다. 어느 한쪽이라도 실패하거나 시간이
끝나면 선택 여부와 무관하게 양쪽 모두 0 A/HALTED로 돌아간다.
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

### 포화 진단

```text
CLAMP           # 도메인 타입별 포화 통계 출력
CLAMP_RESET     # 통계 초기화 (시험 구간을 나눌 때)
```

`Amp`, `Percent`, `Pct0to100`, `Unit`, `Rpm` 각각에 대해 상한/하한에 몇 번
걸렸는지, 잘리기 전 raw 값이 얼마였는지, 어느 소스 위치에서 잘렸는지를
출력한다. 호출 위치는 `__builtin_FILE/LINE/FUNCTION`으로 잡으므로 별도
계측 코드가 필요 없다.

```text
[CLAMP] Amp       high=37 low=0
          worst high raw=812.40 @ src/core/app_wiring.cpp:330 drive_supervisor_update()
          last       raw=523.10 @ src/core/app_wiring.cpp:330 drive_supervisor_update()
[CLAMP] Percent   clean
```

`Amp` 포화 통계는 일반 제어 경로 진단용이다. 이 브랜치의 시리얼 펄스는 별도로
모터당 10 A에서 제한된다.

수락 시 `[MOTOR_TEST] accepted`, 거절 시 원인이 출력된다. 별도 소프트웨어 즉시
정지 명령은 두지 않았다. 모든 명령은 최대 300 ms 안에 자동 종료되며 실제 즉시 정지는
차량의 물리 스위치를 사용한다.

## 로그 판독

- 평상시에는 `STAT`, `MCU`, `CAN` 세 줄 요약을 1 Hz로 출력한다.
- 모터 또는 동기화 시험 중에는 `FAST_CSV` 한 줄을 20 Hz로 출력한다.
- `hs`, `fb`, `age1`, `age2`: 좌·우 핸드셰이크/피드백과 마지막 수신 이후 ms
- `err`: 컨트롤러 error1/2/3
- `state`, `txFail`, `rxMiss`, `busErr`, `arbLost`: ESP32 CAN 드라이버 상태/누적 오류
- `q`, `peak`: 32프레임 RX 큐의 현재 적재량과 부팅 이후 최대 관측 적재량
- `MCU Ibus/Iph/rpm`: 좌·우 컨트롤러 실측값
- `FAST_CSV`의 `cmd_l/r`, `out_l/r`: 요청값과 실제 CAN life-task 송신값
- `FAST_CSV`의 `wss_fl/fr/rl/rr`, `pulse_fl/fr/rl/rr`: 네 WSS의 RPM과 누적 펄스
- 시리얼 펄스에 한해 내부적으로 Drive 명령을 사용하며 실제 GPIO32 기어값은 무시
- `IMU=ok|STALE`: MTi-320 데이터 freshness
- `WSS`, `pulses`: 네 바퀴 RPM과 부팅 이후 누적 상승엣지. 한 바퀴당 24 증가 예상

`rxMiss`는 누적값이므로 재부팅 직후 0에서 증가하는지 확인한다. 짧은 시험 중에도
계속 증가하거나 `peak`가 32에 가까워지면 시험 전류를 올리지 않는다.

PCB V3 WSS 핀은 FL/FR/RL/RR = GPIO18/17/16/4다.

## 권장 순서

1. HV를 끈 채 VCU를 켜고 부팅 로그에서 `[BENCH] ... normal throttle drive DISABLED`를 확인한다.
2. 각 바퀴를 손으로 한 바퀴 돌려 대응 채널 `pulses`가 약 24 증가하는지 확인한다.
3. CAN과 컨트롤러 전원을 연결하고 좌·우 모두 `hs=1/1`, `fb=1/1`, `err=000000/000000`인지 확인한다.
4. 바퀴를 확실히 띄우고 물리 비상정지를 즉시 조작할 수 있게 한다.
5. `MOTOR_L 10 300`, `MOTOR_R 10 300`, `MOTOR_BOTH 10 300` 순서로 한 번씩 실행한다.
6. `FAST_CSV`에서 명령/상전류/RPM/WSS와 CAN 오류 누적 여부를 비교한다.

CAN 오류 카운터가 증가하거나 WSS 채널과 실제 바퀴 위치가 다르면 반복 구동하지
말고 배선과 종단저항부터 확인한다. 이 브랜치에서는 10 A보다 높일 수 없다.
