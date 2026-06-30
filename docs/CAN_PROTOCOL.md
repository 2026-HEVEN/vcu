# HEVEN CAN 프로토콜 명세서 (단일 출처)

> **이 문서는 VCU·Cluster 양 레포가 공유하는 단일 출처(single source of truth)입니다.**
> 코드(`include/can_protocol.h`)와 이 문서는 항상 일치해야 하며, 수정 시 **양 레포에 동일하게 반영**합니다. (담당: 김도현)
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
| Cluster → VCU | 커맨드 (config/리셋) | `0x1801D0C0` (신규) | — | 100ms | 6 |

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

### 5.7 Cluster → VCU : 커맨드  `0x1801D0C0` (신규 할당) · 100ms

> 계기판 스위치로 주행모드/리셋 등을 VCU에 전달. **EZkontrol 표준이 아닌 HEVEN 자체 정의.**
> PF=0x01, PS=0xD0(VCU), SA=0xC0(Cluster). MCU→VCU(0x1801D0EF)와 SA로 구분되어 충돌 없음.

| 바이트 | 항목 | 의미 (제안 — 팀 확정 필요) |
|--------|------|----------------------------|
| 0 | Command ID | 0:NOP, 1:주행모드 설정, 2:에러리셋, 3:캘리브레이션 |
| 1 | Param | 예) 주행모드 0:Normal,1:Efficiency,2:Sport |
| 2~6 | 예약 | 0 |
| 7 | Life signal | 0~0xFF |

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

## 7. ⚠️ 미결정: 계기 데이터 수신 경로

EZkontrol 컨트롤러는 **METER 모드**일 때 피드백을 `0x1801xxEF`(VCU용)가 아니라
`0x180117EF`/`0x180217EF`(METER용, §5.5/5.6)로 보낸다. 현재 동방 테스트 컨트롤러가 METER 모드로 설정돼 있음.

두 가지 경로 중 하나를 팀이 확정해야 한다:

| 경로 | 설명 | 장단점 |
|------|------|--------|
| **A. Cluster = METER 직수신** | Cluster를 SA=0x17(METER)로 동작시켜 MCU 계기 프레임을 직접 받음 | 컨트롤러 재설정 불필요, but VCU가 피드백을 따로 받으려면 컨트롤러를 VCU모드로도 보내게 설정해야 함(이중) |
| **B. VCU 게이트웨이** | 컨트롤러를 VCU모드로 설정 → VCU가 0x1801/0x1802 수신 후, Cluster용 차량상태 프레임을 CAN으로 재방송 | VCU 펌웨어가 단일 진실원, but 컨트롤러 EZkontrol 앱 재설정 필요 |

> 관련 메모: "Can_driver 테스트 컨트롤러는 METER 모드. 제어하려면 앱 재설정 필요."
> → **B로 가려면 컨트롤러를 VCU모드로 재설정**해야 함. 결정 후 이 절을 확정하고 `state` 채우는 RX 파싱을 그에 맞게 구현.

---

## 8. 코드 매핑 (`include/can_protocol.h`)

현재 구현된 것:
```cpp
constexpr uint32_t CAN_ID_TORQUE_L = 0x0C01EFD0;  // VCU→MCU1
constexpr uint32_t CAN_ID_TORQUE_R = 0x0C01F0D0;  // VCU→MCU2
uint16_t torque_to_raw(float amps);   // (amps+3200)*10
float    raw_to_torque(uint16_t raw);
// SA 상수: SA_VCU=0xD0, SA_CLUSTER=0xC0, SA_CONTROLLER_L=0xEF, SA_CONTROLLER_R=0xF0, SA_ENERGY_METER=0x17
```

아직 코드에 추가해야 할 것 (이 문서 기준):
- 피드백 ID 상수: `CAN_ID_FB1_L=0x1801D0EF` 등
- METER ID 상수 (경로 A 선택 시)
- `CAN_ID_CLUSTER_CMD = 0x1801D0C0`
- 디코딩 헬퍼: `raw_to_voltage` (×0.1), `raw_to_current` (×0.1, −3200), `raw_to_temp` (−40), 속도(경로별 1 또는 0.1 rpm/bit)

---

## 9. 변경 관리

- 이 파일과 `include/can_protocol.h`는 **글자 단위로 일치**해야 한다.
- 수정 시: ① 이 문서 갱신 → ② 양 레포의 `can_protocol.h` 동기화 → ③ 변경 요약을 팀 공지.
- 새 메시지 ID는 J1939 규칙(PF/PS/SA)에 맞게 할당하고 §4 표에 추가.
