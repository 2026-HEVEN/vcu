# Car Check CAN 인수인계 — VCU / Cluster / TMA-1
작성: 2026-09-08. 계약 버전 1. 양쪽 저장소의 이 문서와 include/car_check_protocol.h는 동일해야 한다.

## 적용 범위와 상태
이번 변경은 VCU 추가 텔레메트리 송신, Cluster 파싱·보관·유효성 조회와 교차시험이다.
Car Check 사분면/상세 화면과 HOME 버튼은 이번 변경에 포함하지 않는다. 계기팀은 아래 상태 API를 상세 화면에 연결한다.
모터 제어·회생 허용·WSS 제어 경로·TV 게인·전력 제한 설정은 변경하지 않았다. 계기용 WSS 이상치 검사는 기존 제어용 롤오버 결함을 고친 것이 아니다.
현재 TV PID 게인은 0, 회생 검증 플래그와 브레이크 센서 설치 플래그는 OFF다. 스위치 요청만 ON이어도 적용 상태는 OFF가 맞다.

## 공통 규약
- Classical CAN, 29-bit Extended ID, Data frame (RTR 금지), DLC 정확히 8, 250 kbit/s.
- 아래 4개 프레임 모두 50 ms / 20 Hz. 같은 scheduler 작업에서 차례로 전송되며 CAN 도착 시점은 동시에 보장되지 않는다.
- 추가 진단 4프레임은 큐 대기 없이 best-effort 송신한다. VCU TX 큐는 16프레임으로 설정하며, 큐 수용 실패는 sensor_telemetry_tx_drops / 1 Hz CAR_CHECK txDrop 로그에 누적한다. 지속 실패 시 Cluster가 STALE을 표시해야 한다.
- 다중 바이트는 Little Endian. signed 값은 16-bit two's complement.
- life는 8-bit 0..255 순환. 조향/IMU/제어상태는 같은 송신 호출의 life를 공유한다. 휠 프레임에는 life가 없다.
- 수신 유효기간은 300 ms 이하. 300 ms 초과는 STALE. 시간 차는 uint32 모듈러 연산.
- 새 프레임 2개 × 20 Hz = 초당 40프레임 추가. 29-bit DLC8 프레임을 약 130~160 bit로 가정하면 약 5.2~6.4 kbit/s(250 kbit/s의 약 2.1~2.6%) 추가이며, 재전송/경합을 포함한 실버스 부하는 별도 측정한다.

## 프레임 목록
| CAN ID | 의미 | 구현 |
|---|---|---|
| 0x1801C0D0 | 기어·브레이크·HV·스로틀·Paddock | 기존 형식 그대로 |
| 0x1803C0D0 | 대표 차속 km/h × 10, byte2 valid | 기존 형식 그대로 |
| 0x1804C0D0 | 정규화 조향 | 기존 byte0..1 유지, byte6..7 확장 |
| 0x1805C0D0 | IMU yaw rate·가속도 X/Y | 기존 byte0..5 유지, byte6..7 확장 |
| 0x1806C0D0 | 개별 휠 차속 크기 FL/FR/RL/RR | 신규 |
| 0x1807C0D0 | VCU 기능 요청·적용 상태·차단 원인 | 신규 |

## 0x1804C0D0 — 조향
| 바이트 | 형식 | 배율 / Offset | 의미 |
|---|---|---|---|
| 0..1 | int16 LE | raw/1000, offset 0 | -1..+1 정규화 조향 |
| 2..5 | 예약 | 0 | 사용 금지 |
| 6 bit0 | bool | 1=유효 | 이번 ADC 기반 계산값 유효 |
| 6 bit7 | bool | 1=새 유효성 규약 존재 | legacy 송신은 0 |
| 6 나머지 | 예약 | 0 | 사용 금지 |
| 7 | uint8 | 1/count | life |

정규화 값이지 degree가 아니다. 현재 ADC12bit × 4를 기존 raw counts 계약에 넣고 중심 8192, counts/unit 4096, invert=false로 변환한다.
현재 부호는 ADC/count 증가 시 +이다. 차량의 좌/우 실물 대응은 별도 교정해야 하며 확정되지 않은 조향각을 표시하지 않는다.
valid는 raw 범위 및 계산 유한성 검사다. 단선·고착·기계적 보정의 건강 상태를 보장하지 않는다. 레일 값도 소프트웨어 범위 내 값일 수 있다.
새 송신기는 무효값 payload를 0으로 보내지만 valid=0이므로 UI는 0으로 표시하지 않는다.
legacy(byte6 bit7=0) 값은 보관할 수 있으나 유효성 미확정(UNKNOWN)으로 표시한다.

## 0x1805C0D0 — IMU
| 바이트 | 형식 | 단위 / 배율 / Offset |
|---|---|---|
| 0..1 | int16 LE | yaw rate deg/s = raw/100, offset 0 |
| 2..3 | int16 LE | accel X g = raw/100, offset 0 |
| 4..5 | int16 LE | accel Y g = raw/100, offset 0 |
| 6 bit0 | bool | yaw rate 값 유효 |
| 6 bit1 | bool | accel X/Y 두 값 모두 유효 |
| 6 bit7 | bool | 새 유효성 규약 존재 |
| 6 나머지 | 예약 | 0 |
| 7 | uint8 | life |

현재 드라이버는 MTi rate-of-turn Z를 rad/s → deg/s, acceleration X/Y를 m/s² → g(9.80665)로 변환하며 축 회전/부호 반전을 추가하지 않는다.
따라서 sensor X/Y/Z 계약이다. 장착 방향에 따른 차량 전방/좌측/상방 및 yaw 양의 방향은 실물 확인 전까지 확정하지 않는다.
유효성은 checksum-valid MTData2에서 해당 XDI 그룹이 실제로 수신된 후 100 ms 이내이며 값이 유한하고 CAN 표현 범위 내인 경우다.
다른 종류의 MTData2 프레임만 들어온다고 yaw/acceleration을 유효로 만들지 않는다. 부팅 직후 미수신도 false다.
yaw와 accel의 개별 age를 판단하므로 일부 그룹만 수신되면 해당 bit만 켜진다. 그룹들이 물리적으로 같은 sample 시각이라는 보장은 없다.
표현 범위 초과/NaN/Inf는 포화한 정상값이 아니라 해당 valid=false, payload=0이다. legacy 유효성 없는 프레임은 UNKNOWN이다.

## 0x1806C0D0 — 개별 휠 차속
| 바이트 | 휠 | 형식 / 단위 |
|---|---|---|
| 0..1 | FL | uint16 LE, km/h = raw × 0.1 |
| 2..3 | FR | 동일 |
| 4..5 | RL | 동일 |
| 6..7 | RR | 동일 |

채널별 raw=0xFFFF는 INVALID, raw=0은 유효한 측정 0이다. 유효 표현 범위 0..6553.4 km/h, offset 0.
센서는 회전 방향을 식별하지 않으므로 후진도 양의 속도 크기다. 모터 RPM이 아니라 각 WSS의 휠 RPM을 환산한다.
계산: wheel_kph = wheel_rpm × 2π × 0.2387 m × 0.06. 실제 상승 에지 24 PPR, 필터 시정수 0.25 s.
유효 조건: PCNT 초기 구성 및 현재 read 성공, dt 1..100 ms, PPR 양수, pulse delta가 6000 wheel RPM의 계산 범위(+1 pulse 양자화 여유) 이내.
계기용 필터는 제어용 필터와 별도다. 무효 sample에서는 계기용 필터를 초기화한다. 기존 rollover의 거대 delta를 정상 속도로 전송하지 않지만 제어 경로 결함은 별도 수정 대상이다.
중요: 정지 중 PCNT가 0을 정상 읽으면 valid=true다. 연결 안 된 센서도 0일 수 있으므로 이 bit는 측정 계산의 유효성이지 센서 연결/정상 건강 진단이 아니다.
프레임을 잃으면 네 채널 모두 STALE이다. 센서가 고착됐는데 프레임만 계속 수신되는 상황은 이 계약의 통신 STALE 검사가 검출하지 못한다.

## 0x1807C0D0 — 요청 / 적용 상태
| 바이트 | 비트 | 의미 |
|---|---|---|
| 0 | 전체 | 버전=1. 다른 버전은 UNKNOWN |
| 1 | bit0 / bit1 / bit2 | VCU가 수신한 TV / 회생 / Paddock 요청 |
| 2 | bit0 | TV active: 실제 TV pipeline 선택 + 일반 출력 허용 + 명령 source fresh |
| 2 | bit1 | 회생 available: 아래 진단 조건을 충족 |
| 2 | bit2 | 회생 active: 회생 요구가 있고 최종 supervisor 전류가 기어 기준 반대 부호 |
| 2 | bit3 | Paddock 실제 제한 모드 active (기존 status bit4와 같은 상태) |
| 2 | bit4 | 일반 출력 허용 관측값. 벤치/동기화 override에서는 false |
| 2 | bit5 | Cluster 명령 수신 fresh |
| 2 | bit6 | 브레이크 센서 설치 설정 |
| 3 | bitmap | TV 비활성 조건 |
| 4 | bitmap | 회생 비활성 조건 |
| 5..6 | 전체 | 예약 0 |
| 7 | 전체 | life |

TV active는 차등 전류가 반드시 0이 아니라는 뜻이 아니며 타이어에 실제 토크가 생겼다는 피드백도 아니다. 직진 오차 0에서도 제어 pipeline은 active일 수 있다.
회생 available 진단 조건: 회생 요청 + Cluster fresh + REGEN_HARDWARE_VALIDATED + BRAKE_SENSOR_INSTALLED + pack_data_valid + 유효 SOC 0..0.95 미만 + 일반 출력 허용.
available은 현재 관측 조건의 요약이며 BMS charge-acceptance의 보증이 아니다. 이 코드가 제어 허용 조건을 새로 열거나 추가 보호를 실제 제어에 적용하지 않는다.
회생 active는 위 조건에 브레이크 요구·음의 longitudinal 회생 요구·양쪽 최종 supervisor 전류가 기어 기준 반대 부호(최소 한쪽 nonzero)를 추가한다.
회생 active도 실제 에너지 회수 측정값이 아니며 최종 CAN 전송 성공의 ACK가 아니다. 회생 부호 소실 결함 때문에 실제 명령이 구동 방향이면 active=false, direction mismatch를 보고한다.
출력 허용 및 active는 scheduler 시점의 진단 관측이며 아직 수정 전인 코어 간 원자적 모터 명령 게시 문제를 해결한 것이 아니다.
Paddock은 Cluster 통신이 끊겨도 이미 적용한 제한을 유지할 수 있으므로 요청 OFF, applied ON, cluster_fresh=false 조합이 가능하다.
회생 스위치 0단=OFF, 1/2/3단=동일 ON. 기존 0x1801D0C0 byte1 bit0 TV, bit1 회생 요청 계약은 바꾸지 않는다.

### byte3 TV 비활성 조건
bit0 요청 OFF, bit1 PID 게인 모두 0, bit2 IMU 무효, bit3 대표 차속 무효, bit4 최소 TV 속도 미만, bit5 일반 출력 차단, bit6 벤치/동기화 override. bit7 예약.
여러 조건을 동시에 표시한다. 조건 존재는 장치 고장과 동의어가 아니다. 상태 active를 request로 대체하면 안 된다.

### byte4 회생 비활성 조건
bit0 요청 OFF, bit1 회생 하드웨어 검증 OFF, bit2 브레이크 센서 미설치, bit3 BMS 무효,
bit4 일반 출력 차단/override, bit5 브레이크 요구 없음, bit6 SOC 무효 또는 95% 이상,
bit7 회생 요구와 최종 전류 방향 불일치.
브레이크를 밟지 않은 available 상태에서는 bit5가 켜져도 고장이 아니다.
현재 미완성 회생 제어에 대한 진단일 뿐, 신규 BMS 충전 안전 설계의 대체품이 아니다.

## Cluster 수신 / 화면 팀 API
state.car_check에 steering, imu, wheels, control 및 프레임별 Reception이 저장된다.
CarCheckReceiver::receive()는 Extended 및 DLC8만 허용한다. 실 CAN 경로에서는 RTR도 거부한다.
- steering_quality(now), yaw_quality(now), accel_quality(now), wheel_quality(ch,now), control_quality(now)
- Quality::Missing → '-' / 아직 수신하지 않음
- Quality::Stale → 'STALE' / 마지막 값이 있어도 정상값처럼 표시 금지
- Quality::Unsupported → 'UNKNOWN' / legacy 유효성 없음 또는 지원하지 않는 버전
- Quality::Invalid → 'INVALID' / 통신은 도착하지만 센서/계산이 무효
- Quality::Valid → 값+단위. 0은 '0'. ON/OFF도 반드시 quality를 먼저 확인.

값 자체는 마지막 수신값을 보관한다. 따라서 raw struct의 bool만 읽지 말고 quality와 함께 사용해야 한다.
요청은 Cluster local switch 상태와 VCU echo(control.*_requested)를 별도로 표시할 수 있다.
적용 상태는 control.tv_active / regen_available / regen_active / paddock_active를 사용한다.
기존 PDK 표시는 0x1801C0D0을 그대로 사용 가능하다. 새 상세 화면에서 이 프레임과 0x1807C0D0 사이의 비동기 갱신을 고장으로 단정하지 않는다.

## 모터 컨트롤러 직접 수신
0x1801D0EF / 0x1801D0F0 (Part I), 0x1802D0EF / 0x1802D0F0 (Part II)는 기존 수신 경로를 유지한다.
이번에 Part I byte4..5 상전류를 무시하지 않고 state.phase_current / phase_current_r에 저장한다.
전압 raw×0.1 V, 버스/상전류 raw×0.1−3200 A, RPM raw−32000; 모두 uint16 LE raw.
Part II 온도 byte0/1 raw−40 °C, status byte2, error byte3..5. 컨트롤러 직접 원본이므로 VCU에서 중복 재송신하지 않는다.
0x55 ×8 handshake probe는 실제 측정으로 파싱하지 않는다. 장치별 Part I / Part II 수신 시각을 따로 확인한 뒤 온도·오류 표시를 사용한다.

## 재현 가능한 테스트
VCU: pio test -e native, pio run -e esp32dev.
Cluster: pio test -e native, pio run -e esp32dev.
VCU 저장소에서:
```text
python tools/check_car_check_contract.py --cluster ../HEVEN-cluster
```
C++17 g++ 또는 clang++ 필요. --cxx로 경로 지정 가능. 실제 VCU encoder와 실제 Cluster decoder/receiver를 함께 링크해 2149개 round-trip 벡터 및 freshness를 검사한다.
각 저장소 native 테스트는 독립적인 고정 HEX 기대값, 부호/단위, 유효 0/무효, legacy, malformed DLC, 시간 wrap, timeout/복구 및 요청과 실제 상태 구분을 검사한다.
이는 소프트웨어 검증이며 실 CAN 부하·실물 축 방향·센서 단선·모터 제동 검증을 대신하지 않는다.

2026-09-08 검증: VCU native 171/171, Cluster native 81/81, 실제 송수신 cross-test 2149 벡터 통과. VCU/Cluster ESP32 빌드 성공. Cluster는 한글 경로의 linker map 오류로 해당 실행에만 PLATFORMIO_BUILD_DIR을 임시 영문 경로로 지정해 검증했다. 공용 platformio.ini에 새 개인 경로를 넣지 않았다. 기존 framebuffer.cpp 들여쓰기 경고는 이번 통신 수정 범위 밖이다.

## 계기팀과 다음 담당자의 남은 작업
- Car Check 사분면·상세 화면·GPIO13 HOME 연결(별도 HMI 작업).
- 실제 조향 부호·IMU 장착 축 확인 후 확정 보정값 공유.
- WSS 제어용 롤오버 및 모터 명령 스냅샷 개선은 별도 이슈로 유지.
- 회생 부호·BMS charge acceptance 검증 전 회생 기능은 계속 OFF.
- 새 펌웨어를 양쪽에 업로드한 뒤 CAN analyzer/TMA-1에서 ID·주기·유효성·timeout 표시 확인.
