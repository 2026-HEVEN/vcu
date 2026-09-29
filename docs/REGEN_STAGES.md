# 회생 단계 / 작동 조건 (2026-09-29)

설정: `src/modules/realcar_calibration.h`, `realcar_cal::bringup`.

| 변수 | 기본값 | 의미 |
|---|---|---|
| REGEN_FOUR_STAGE_ENABLED | true | false이면 로터리 1/2/3 모두 3단계 전류 사용 |
| REGEN_ONE_PEDAL_ENABLED | true | false이면 스로틀 해제에 더해 설치된 브레이크 ON 필요 |
| REGEN_LEVEL1_TOTAL_CURRENT_A | 100 | 양쪽 요구 상전류 합계 A |
| REGEN_LEVEL2_TOTAL_CURRENT_A | 200 | 양쪽 요구 상전류 합계 A |
| REGEN_LEVEL3_TOTAL_CURRENT_A | 300 | 양쪽 요구 상전류 합계 A |
| REGEN_EFF_TOTAL_CURRENT_CAP_A | 30 | 기존 Efficiency 모드의 별도 회생 상한 보존; Paddock과 다름 |

0단은 항상 OFF. 전류는 버스/충전전류가 아니며 TV 배분·우측 보정·최종 제한 전 요구값이다.
기존 REGEN_TOTAL_CURRENT_NORMAL_A/EFF_A는 위 단계값/효율모드 상한으로 대체했다.
스로틀 0% 연속 100ms 조건 유지. 양의 스로틀은 즉시 회생을 취소하며 브레이크와 함께 밟아도 회생하지 않는다.
브레이크 모드는 BRAKE_SENSOR_INSTALLED=true여야 한다. 브레이크 해제 시 회생을 중단한다.
BMS 유효·SOC taper(90~95%)·D기어·좌우 전진 RPM·통신/최종 명령 허용 조건은 그대로 유지한다.

## CAN 확장

Cluster → VCU `0x1801D0C0`, Extended, DLC 8. 기존 주기 유지.
- Byte1 bit1: 기존 회생 master ON.
- Byte3: `0xA0 | rotary_level`, 로터리 0..3 → A0/A1/A2/A3.
- VCU는 master OFF이면 단계도 0. master ON + 명시 단계 0도 OFF.
- Byte3=0인 구형 Cluster는 ON을 3단계로 해석한다(기존 일반 회생 강도 보존).
- 그 외 잘못된 Byte3는 회생 OFF. 통신 timeout에도 요청·단계를 0으로 초기화.
- TV/Debug/Paddock 비트는 변경하지 않았다.

Cluster는 모드를 판단하지 않고 물리 로터리 위치를 항상 송신한다.
구형 VCU는 단계 확장을 무시하여 1/2/3 모두 기존 ON으로 작동한다.
따라서 **단계 시험 전에 VCU와 Cluster를 모두 새 펌웨어로 업로드해야 한다.**
현재 계기 회생 표시는 기존 ON/OFF 표시이며 적용 단계 확인은 CAN Byte3와 목표 전류로 한다.
새 단계 값은 시험 설정이지 배터리 충전 허용 전류 검증 결과가 아니다.

보드 업로드·커밋·푸시는 별도 요청 전 수행하지 않는다.
