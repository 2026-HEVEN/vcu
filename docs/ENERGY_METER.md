# EM Gateway 수신 및 전력 제한 — energymeter-jys

## 기준 및 범위

- 원격 VCU dev `5cd457d`에서 분기. PR #30을 병합하지 않고 Cluster와 맞춰 구현.
- Cluster dev `8a5bfc6`의 include/can_protocol.h, src/core/can_bus.cpp,
  src/logic/can_protocol.cpp에 있는 실제 디코딩 정의를 대조했다.
- 원래 HEVEN-vcu 폴더의 미커밋 회생/스로틀 변경은 이 브랜치에 포함하지 않는다.
- 계기판 코드 수정이나 EM 데이터 재송신은 필요 없다. 같은 버스의 프레임을 수신한다.

## CAN 계약

250 kbps, Extended data frame, DLC 8, 송신 SA 0xC1.
RECORD = **0x1CF5FFC1** (문서상 10ms), SYNC = **0x1CF6FFC1** (예약, 전력 신선도에 사용 안 함).
0x180117D0 또는 EZkontrol의 METER 주소 0x17과 혼동하지 않는다.

| 바이트 | 타입 | 배율 | 값 |
|---|---|---|---|
| 0–1 | signed int16 little-endian | 0.1 V | HV 전압 |
| 2–3 | signed int16 little-endian | 0.1 A | HV 전류 |
| 4–5 | signed int16 little-endian | 0.01 V | LV 전압 |
| 6–7 | signed int16 little-endian | 0.01 °C | EM CPU 온도 (배터리 온도 아님) |

signed 전력 = HV 전압 × HV 전류. Cluster 기준 방전 +, 충전 -를 보존한다.
오프셋을 추가하지 않는다. 예: `F4 01 9C FF D2 04 0C FE` →
50V, -10A, 12.34V, -5°C, **-500W**. 327kW로 오독하지 않는다.
실제 센서의 설치 방향/전류 부호는 별도 실측으로 확인해야 한다.

## 수신 유효성

- 다른 ID/SYNC는 전력 타임스탬프를 갱신하지 않는다.
- RECORD의 Standard/RTR/잘못된 DLC는 거부하고 기존 샘플도 제어용 무효 처리.
- `seen`으로 미수신과 실제 0을 구분. 시간 0의 정상 수신도 허용한다.
- HV≤0은 측정값으로 남기되 구동 허용용으로는 무효.
- **100ms 초과** 미수신이면 stale (uint32 시간 rollover 지원).
  Cluster의 표시용 500ms와 달리 제어용 100ms를 선택했으며 실차 버스 부하 시험이 필요하다.
- 수신 시각은 VCU가 큐에서 꺼낸 시각이다. 센서 측 시각/sequence/센서 내부 freeze는
  RECORD 계약에 없어 검출하지 못한다. 임의 sentinel/CRC를 만들지 않았다.

## 전력 제어

`realcar_calibration.h`:

- `ENABLE_ENERGY_METER_LIMIT=false`: 수신·진단은 항상 수행, 제어는 기존 그대로.
- `ENERGY_METER_STALE_MS=100`: 제어용 freshness.
- 기존 `DRIVE_POWER_SOFT_LIMIT_W=8000` 유지. 새 별도 10kW 상한을 중복 도입하지 않는다.

활성화 시 governing power = max(기존 컨트롤러 전력, 기존 실측 상전류 모델,
기존 명령 예측 전력, **max(EM signed power,0)**).
기존 전류 스케일링·상승률·온도·Paddock 보호를 유지한다.
유효한 음수 EM 전력은 소비전력 0으로 취급하고 충전량으로 표시한다.
회생 충전전류 제한이나 BMS 충전 허용 권한을 이 값으로 대신하지 않는다.

제한을 켠 상태에서 미수신/무효/stale이면 **구동 전류를 0으로 제한**한다.
단순히 EM을 제외한 추정값으로 돌아가며 출력이 커지는 fallback은 하지 않는다.
후진 구동도 차단하며 회생 경로의 기존 정책은 바꾸지 않는다.
이를 위해 구동 판정은 보정 스로틀 >0%로 한다(기존 >1%에서는 미소 후진
전류가 구동으로 분류되지 않았다). 기어 진입용 1% 해제 기준 및 브레이크 정책은 유지한다.
정상 수신 복귀 후 구동은 기존 가속 램프로 0부터 복귀한다(스로틀 재해제 latch는 없음).
센서 누락에 의한 구동 차단은 새 정책이므로 활성화 전 반드시 운전자와 공유한다.

## 관찰 및 시험

1Hz 시리얼 `EM` 행: seen/fresh/age/rx/reject, HV/I/P/LV/CPU, limit/blocked.
미수신 시 age=4294967295; 수치가 초기 0이어도 seen=0이면 실제 측정값이 아니다.
부호·단위·실제 갱신 주기·Cluster 값 일치를 확인한 뒤 제한을 켠다.
0V/양전류/음전류, 통신 끊김, 복귀 램프를 저출력 시험한다.
이 피드백 제한만으로 순간 전력이 규정 10kW를 절대 넘지 않는다고 보장하지 않는다.

native test_energy_meter는 프레임/부호/타임아웃/rollover/제어 OFF/ON/
stale 차단/복귀 램프/기존 보호 보존을 검증한다. 보드 업로드는 별도다.
