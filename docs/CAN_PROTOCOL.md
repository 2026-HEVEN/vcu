# HEVEN CAN 프로토콜 명세서 (단일 출처)

> **CAN ID와 wire layout은 VCU·Cluster·Monolith가 공유하는 계약입니다.**
> 양 레포의 구현 헬퍼까지 글자 단위로 같을 필요는 없지만, 버스에 보이는 ID·bit·scaling을
> 바꿀 때는 세 시스템을 함께 갱신합니다. (담당: 김도현)
> 출처: `EZkontrol-CANBUS-MCU-to-VCU.pdf V1.0`, `EZkontrol-CANBUS-MCU-to-METER.pdf V1.1`, 시스템 설계.

---

## 1. 버스 파라미터

| 항목 | 값 |
|------|-----|
| 비트레이트 | **250 kbps** |
| 프레임 | CAN 2.0B **확장(29비트) 식별자** |
| 바이트 순서 | **리틀엔디언** (예: 0x1234 → 0x34 먼저, 0x12 나중) |
| 토폴로지 | 데이지체인, 양 끝단 120Ω 종단 |
| 표준 | SAE J1939 기반 |

---

## 2. 29비트 식별자 구조 (J1939)

```
 28..26   25   24   23..16     15..8       7..0
┌────────┬────┬────┬────────┬──────────┬────────────┐
│PRIORITY│ R  │ DP │  PF    │   PS     │     SA     │
│ 3bit   │ 0  │ 0  │ 8bit   │  8bit    │   8bit     │
└────────┴────┴────┴────────┴──────────┴────────────┘
```
- **PRIORITY**: 0~7 (값 작을수록 우선). 제어명령=3, 피드백=6.
- **PF (PDU Format)**: 메시지 종류. PF<240이면 PS=목적지 주소(DA), PF≥240이면 PS=그룹확장.
- **PS**: 목적지 노드 주소 (PF<240일 때).
- **SA**: 송신 노드 주소.

> 예) `0x0C01EFD0` → PRIORITY=3, PF=0x01, PS=0xEF(MCU1 목적지), SA=0xD0(VCU 송신).
> 즉 **"VCU가 MCU1에게 보내는 1번 메시지"**.

---

## 3. 노드 주소 (SA)

### HEVEN 시스템에서 실제 사용

| 노드 | SA (HEX) | SA (DEC) | 비고 |
|------|----------|----------|------|
| Energy_Meter / **METER** | 0x17 | 23 | EZkontrol 표준 계기 주소 |
| **Cluster_ESP32** | 0xC0 | 192 | 계기판 HMI (시스템 설계) |
| **VCU_ESP32** | 0xD0 | 208 | 차량 제어 |
| **Controller_L (MCU1)** | 0xEF | 239 | 좌측 모터컨트롤러 (기본값) |
| **Controller_R (MCU2)** | 0xF0 | 240 | 우측 — **EZkontrol 앱에서 SA=0xF0 수동 설정 필요** |

### EZkontrol 표준 전체 주소표 (참고)

| 노드 | 주소 |
|------|------|
| METER | 23 (0x17) |
| VCU | 208 (0xD0) |
| MCU1~4 | 239~242 (0xEF~0xF2) |
| BMS1~4 | 243~246 (0xF3~0xF6) |
| GLOBAL | 255 (0xFF) |

> **N.B.** MCU의 SA는 호스트(EZkontrol 앱)에서 "controller number"로 설정. 기본 SA = 239(0xEF).
> Controller_R은 반드시 240(0xF0)으로 바꿔야 좌우가 충돌하지 않음.

---

## 4. 메시지 ID 일람 (HEVEN)

| 방향 | 메시지 | Controller_L (0xEF) | Controller_R (0xF0) | 주기 | Prio |
|------|--------|---------------------|---------------------|------|------|
| VCU → MCU | 제어명령 / 핸드셰이크응답 | `0x0C01EFD0` | `0x0C01F0D0` | 50ms | 3 |
| MCU → VCU | 피드백 Part I (전압/전류/속도) | `0x1801D0EF` | `0x1801D0F0` | 50ms | 6 |
| MCU → VCU | 피드백 Part II (온도/상태/에러) | `0x1802D0EF` | `0x1802D0F0` | 50ms | 6 |
| MCU → METER | 계기 메시지 I | `0x180117EF` | `0x180117F0` | 100ms | 6 |
| MCU → METER | 계기 메시지 II | `0x180217EF` | `0x180217F0` | 100ms | 6 |
| Cluster → VCU | 커맨드 (TC 표기의 TV enable/Regen Auto/Debug/Paddock) | `0x1801D0C0` (신규) | — | ~20ms | 8 |
| VCU → Cluster | 기어/브레이크/HV 상태 | `0x1801C0D0` (신규) | — | 50ms | 6 |
| VCU → Cluster/TMA-1 | 단일 차량속도 | `0x1803C0D0` (신규) | — | 50ms | 6 |
| VCU → TMA-1 | 조향 텔레메트리 | `0x1804C0D0` (신규) | — | 50ms | 6 |
| VCU → TMA-1 | IMU 텔레메트리 | `0x1805C0D0` (신규) | — | 50ms | 6 |
| Cluster → logger | BMS 상태 요약 | `0x18F3FFC0` | — | 100ms | 6 |

> ID에서 PS(목적지)·SA(송신)만 컨트롤러별로 바뀜. 위 표의 ID는 `PF<<16 | PS<<8 | SA`로 조립됨(+ Priority).

---

## 5. 메시지 상세

### 5.1 VCU → MCU : 제어명령  `0x0C01EFD0` (L) / `0x0C01F0D0` (R) · 50ms

| 바이트 | 항목 | 분해능 | 오프셋 | 범위 |
|--------|------|--------|--------|------|
| 0~1 | Target Phase Current (토크) | 0.1 A/bit | −3200 A | −3200~3200 A |
| 2~3 | Target Speed | 1 rpm/bit | −32000 | −32000~32000 rpm |
| 4 | Command Controls | — | — | bit0: 0=HALTED / 1=RUNNING<br>bit1: 0=토크모드 / 1=속도모드<br>bit7-2: 예약 |
| 5,6 | 예약 | — | — | 0 |
| 7 | Life signal | — | — | 0~0xFF (매 프레임 +1) |

**토크 스케일링**: `raw16 = (target_A + 3200) × 10` (리틀엔디언). 음수 = 회생제동.
→ 0A=32000, +32A=32320, −32A=31680. **`include/can_protocol.h`의 `torque_to_raw()`와 동일.**

> 시험 전용 시간축 동기화도 이 기존 프레임에 좌·우 동일 전류 펄스를 실어
> Energy Meter HV Current와 Monolith 컨트롤러 전류 로그를 정합한다. 별도 CAN
> ID는 추가하지 않는다. 실행 조건과 절차는 `docs/TIME_SYNC_PULSE.md`를 따른다.

### 5.2 VCU → MCU : 핸드셰이크 응답  `0x0C01EFD0` / `0x0C01F0D0`

8바이트 **전부 `0xAA`**. (MCU의 0x55 요청에 대한 응답 — §6 참고)

### 5.3 MCU → VCU : 피드백 Part I  `0x1801D0EF` (L) / `0x1801D0F0` (R) · 50ms

| 바이트 | 항목 | 분해능 | 오프셋 | 범위 |
|--------|------|--------|--------|------|
| 0~1 | Bus Voltage | 0.1 V/bit | 0 | 0~300 V |
| 2~3 | Bus Current | 0.1 A/bit | −3200 A | −3200~3200 A |
| 4~5 | Phase Current | 0.1 A/bit | −3200 A | −3200~3200 A |
| 6~7 | Speed | **1 rpm/bit** | −32000 | −32000~32000 rpm |

### 5.4 MCU → VCU : 피드백 Part II  `0x1802D0EF` (L) / `0x1802D0F0` (R) · 50ms

| 바이트 | 항목 | 분해능 | 오프셋 |
|--------|------|--------|--------|
| 0 | Controller Temperature | 1 ℃/bit | −40 ℃ |
| 1 | Motor Temperature | 1 ℃/bit | −40 ℃ |
| 2 | STATUS — bit0: HALTED/RUNNING, bit1: 토크/속도모드 | — | — |
| 3 | ERROR1 비트맵 (아래) | — | — |
| 4 | ERROR2 비트맵 (아래) | — | — |
| 5 | ERROR3 비트맵 (아래) | — | — |
| 6 | 예약 | — | — |
| 7 | Life signal | — | 0~0xFF |

**ERROR 비트맵** (각 비트 0=정상 / 1=에러):
- **Byte3**: 0 과전류, 1 과부하, 2 과전압, 3 저전압, 4 컨트롤러과열, 5 모터과열, 6 모터스톨, 7 모터결상
- **Byte4**: 0 모터센서, 1 모터보조센서, 2 엔코더정렬불량, 3 폭주방지작동, 4 메인가속, 5 보조가속, 6 프리차지, 7 DC컨택터
- **Byte5**: 0 전력밸브, 1 전류센서, 2 오토튠, 3 RS485, 4 CAN, 5 소프트웨어

### 5.5 MCU → METER : 계기 메시지 I  `0x180117EF` · 100ms

> Cluster가 METER(0x17)로 동작할 때 직접 수신 (§7 경로 결정 참고).

| 바이트 | 항목 | 분해능 | 오프셋 |
|--------|------|--------|--------|
| 0~1 | Bus Voltage | 0.1 V/bit | 0 |
| 2~3 | Bus Current | 0.1 A/bit | −3200 A |
| 4~5 | Phase Current | 0.1 A/bit | −3200 A |
| 6~7 | Speed | **0.1 rpm/bit** ⚠️ | −32000 |

> ⚠️ **METER 경로의 Speed는 0.1rpm/bit** — VCU 경로(5.3, 1rpm/bit)와 분해능이 다르다. 혼동 주의.
> (PDF 본문의 PS 표기 "0x11"은 오타이며, ID값 `0x180117EF`가 정답 — PS=0x17=METER.)

### 5.6 MCU → METER : 계기 메시지 II  `0x180217EF` · 100ms

| 바이트 | 항목 | 분해능/의미 |
|--------|------|-------------|
| 0 | Controller Temperature | 1 ℃/bit, offset −40 |
| 1 | Motor Temperature | 1 ℃/bit, offset −40 |
| 2 | Accelerator Opening | 1 %/bit (0~100%) |
| 3 | STATUS — bit2-0 Gear(0:N/A,1:R,2:N,3:D1,4:D2,5:D3,6:S,7:P), bit3 Brake(0/1), bit6-4 Operation Mode(2:Cruise,3:EBS,4:Hold), bit7 DC Contactor(0:OFF/1:ON) | — |
| 4 | ERROR1 (5.4 Byte3과 동일 비트맵) | — |
| 5 | ERROR2 (5.4 Byte4와 동일) | — |
| 6 | ERROR3 (5.4 Byte5와 동일) | — |
| 7 | bit7-4: Life signal | 0~0xFF |

### 5.7 Cluster → VCU : 커맨드  `0x1801D0C0` (신규 할당) · ~20ms

> 계기판 config 입력(TC·Regen Auto·Debug·Paddock)을 VCU에 전달. **EZkontrol 표준이 아닌 HEVEN 자체 정의.**
> 현재 프로젝트에서 `TC`라는 물리/UI 표기는 별도 traction control이 아니라 기존
> torque vectoring(TV) 활성 요청을 뜻한다.
> PF=0x01, PS=0xD0(VCU), SA=0xC0(Cluster). MCU→VCU(0x1801D0EF)와 SA로 구분되어 충돌 없음.
> VCU 디코딩 구현 기준: `decode_cluster_command()`.

| 바이트 | 항목 | 의미 |
|--------|------|------|
| 0 | Reserved | 0 |
| 1 | Config flags | bit0: TC-labelled TV enable request, bit1: Regen Auto request, bit2: reserved(0), bit3: Debug request, bit7-4: reserved(0) |
| 2 | Flags | bit0: Paddock request, bit7-1: reserved(0) |
| 3~7 | 예약 | 0 |

`Regen Auto request` 해석:

| 값 | 의미 |
|----|------|
| 0 | 회생제동 OFF 요청 |
| 1 | VCU 자동 회생제동 허용 요청 |

> ⚠️ TV enable/Paddock/Regen Auto/Debug는 모두 **요청 신호**다. TV는 IMU 유효성·최소속도·PID 게인 조건을 모두 통과해야 실제 차등전류를 만든다. Paddock 진입 조건과 회생 전류/차단 여부도 VCU가 최종 판단한다.
> Debug bit는 유지하되, VCU가 현재처럼 Serial debug를 항상 출력한다면 무시해도 된다. 단, 파싱 시 bit 위치는 보존한다.

### 5.8 VCU → Cluster : 표시 상태 `0x1801C0D0` (HEVEN 정의) · 50ms

| 바이트 | 항목 | 의미 |
|--------|------|------|
| 0 | Gear | 0=N, 1=R, 2=D, 3=P |
| 1 | Flags | bit0 Brake, bit1 HV active, bit2 SOC valid |
| 2 | SOC | 0~100, bit2가 1일 때만 유효 |
| 3~6 | 예약 | 0 |
| 7 | Life | 0~255 |

현재 VCU는 Cluster가 BLE BMS를 직접 표시하므로 SOC valid를 0으로 보낸다.
기어 셀렉터 실측 전 bring-up 프로파일에서는 전진 고정 상태를 D로 송신한다.
HV active는 좌·우 컨트롤러 피드백이 fresh이고 DC bus가 20V를 넘을 때 1이다.

### 5.9 VCU → Cluster/TMA-1 : 단일 차량속도 `0x1803C0D0` (HEVEN 정의) · 50ms

> VCU의 `vehicle_speed_compute()`가 산출한 단일 차량속도를 Cluster LCD 표시와 TMA-1 Control Hub 그래프/로깅용으로 전달한다.
> 개별 4채널 WSS RPM은 VCU 내부 계산용으로만 사용하며, CAN telemetry로 내보내지 않는다.

| 바이트 | 항목 | 분해능/의미 |
|--------|------|-------------|
| 0~1 | Vehicle speed | uint16 little-endian, km/h x 10 |
| 2 | Valid flag | 1=valid, 0=invalid |
| 3~7 | Reserved | 0 |

구현 위치:
- 인코딩: `encode_vcu_vehicle_speed()`
- 송신: `can_bus::send_vehicle_speed()`
- 주기: `app_wiring.cpp` scheduler에서 50ms, 20Hz

### 5.10 VCU → TMA-1 : 조향 `0x1804C0D0` · 50ms

- Byte 0~1: signed int16 little-endian, 정규화 조향값 ×1000
- Byte 2~7: 0

### 5.11 VCU → TMA-1 : IMU `0x1805C0D0` · 50ms

- Byte 0~1: yaw rate [deg/s] ×100, signed int16 little-endian
- Byte 2~3: accel X [g] ×100, signed int16 little-endian
- Byte 4~5: accel Y [g] ×100, signed int16 little-endian
- Byte 6~7: 0

### 5.12 Cluster → logger/VCU 진단 : BMS 상태 `0x18F3FFC0` · 100ms

VCU도 이 프레임을 관찰해 시리얼 진단 상태에 보관하지만 **안전 제어 입력으로
사용하지 않는다**. 현재 경로는 BLE 원천 약 1Hz이고 Cluster 파서와 전류 부호가
실차에서 완전히 검증되지 않았기 때문이다.

---

## 6. 핸드셰이크 & 타임아웃 (EZkontrol 규칙)

```
[전원 ON]
   │  MCU가 0x1801D0EF 로 8바이트 0x55 를 50ms(20Hz)마다 송신
   ▼
VCU가 0x55 수신 ──► VCU가 0x0C01EFD0 로 8바이트 0xAA 응답
   │
   ▼  핸드셰이크 성립
MCU: 메시지 I·II 송신 시작 + VCU 제어명령(메시지 I) 대기·실행
```

**통신 실패 판정 (MCU 측):**
- VCU 제어명령(0x0C01EFD0)을 **10회 연속** 못 받거나
- Life signal **5회 연속** 실패

→ MCU 셧다운 후 핸드셰이크 재시도. (그래서 VCU는 50ms 라이프/제어 프레임을 **절대 끊지 말 것** — 펌웨어의 전용 FreeRTOS 라이프 태스크가 이를 보장.)

---

## 7. 확정: VCU 모드 피드백을 Cluster와 TMA-1이 스니핑

두 컨트롤러를 VCU 모드로 설정하고, 좌측 SA는 `0xEF`, 우측 SA는 `0xF0`을
사용한다. VCU는 `0x1801D0EF/F0`, `0x1802D0EF/F0`을 직접 수신해 제어 감독에
사용한다. Cluster와 TMA-1은 같은 멀티드롭 CAN 버스에서 해당 프레임을 읽기만
하며, VCU가 컨트롤러 피드백을 별도 게이트웨이 프레임으로 재방송하지 않는다.

---

## 8. 코드 매핑 (`include/can_protocol.h`)

현재 구현된 것:
```cpp
constexpr uint32_t CAN_ID_TORQUE_L = 0x0C01EFD0;  // VCU→MCU1
constexpr uint32_t CAN_ID_TORQUE_R = 0x0C01F0D0;  // VCU→MCU2
constexpr uint32_t CAN_ID_VCU_VEHICLE_SPEED = 0x1803C0D0; // VCU→Cluster/TMA-1 speed
constexpr uint32_t CAN_ID_VCU_CLUSTER_STATUS = 0x1801C0D0;
constexpr uint32_t CAN_ID_VCU_STEERING = 0x1804C0D0;
constexpr uint32_t CAN_ID_VCU_IMU = 0x1805C0D0;
constexpr uint32_t CAN_ID_CLUSTER_BMS_STATUS = 0x18F3FFC0;
uint16_t torque_to_raw(float amps);   // (amps+3200)*10
float    raw_to_torque(uint16_t raw);
uint16_t vehicle_speed_kph_to_raw(float kph);
void     encode_vcu_vehicle_speed(float speed_kph, bool valid, uint8_t out[8]);
// SA 상수: SA_VCU=0xD0, SA_CLUSTER=0xC0, SA_CONTROLLER_L=0xEF, SA_CONTROLLER_R=0xF0, SA_ENERGY_METER=0x17
```

현재 `dev`에는 좌·우 Part I/II 디코더, Cluster 명령 디코더, 상태/차속/조향/IMU
인코더가 모두 구현돼 있다. METER 경로는 사용하지 않는다.

---

## 9. 변경 관리

- ID·byte·bit·scaling 계약은 양 레포와 Monolith decoder에서 동일해야 한다.
- 수정 시: ① 이 문서 갱신 → ② 필요한 송수신 레포 갱신 → ③ Monolith decoder 갱신 → ④ 팀 공지.
- 새 메시지 ID는 J1939 규칙(PF/PS/SA)에 맞게 할당하고 §4 표에 추가.
