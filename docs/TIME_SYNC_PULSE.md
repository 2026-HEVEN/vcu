# Energy Meter–Monolith 시간축 동기화 펄스

Energy Meter의 100 Hz HV Current와 Monolith에 저장되는 좌·우 EZkontrol
Bus Current/Phase Current의 시간축을 상관 정합하기 위한 시험 전용 기능이다.
새 CAN ID를 만들지 않고 기존 좌·우 Target Phase Current 명령을 사용한다.

## 안전 동작

- 전원 인가나 Drive 진입만으로는 절대 자동 실행되지 않는다.
- USB 시리얼에서 `SYNC_ARM` 후 10초 안에 `SYNC_RUN`을 입력해야 한다.
- `SYNC_CANCEL`은 arm 또는 실행 중인 파형을 즉시 취소한다.
- 양쪽 컨트롤러 핸드셰이크, 네 종류 Part I/II 피드백 fresh, 고장 없음,
  Drive 안전상태, D 기어, 스로틀 해제, 브레이크 해제 조건이 필요하다.
- TV, Regen Auto, Paddock 요청이 모두 꺼져 있어야 한다.
- 전륜 WSS 기반 차량속도가 유효하고 1 km/h 이하여야 하며, 실행 중에도
  이 조건을 벗어나면 중단한다. 따라서 지면 주행 중 실행을 계속할 수 없다.
- 실행 중 조건 하나라도 깨지면 다음 10 ms 제어 주기에서 0 A로 중단한다.
- 펄스 요청은 기존 drive supervisor를 통과하므로 컨트롤러 stale/fault와
  열 제한은 그대로 적용된다.

## 기본 파형

- 좌·우 동시 20 A/모터
- 0.5초 ON / 0.5초 OFF × 3회
- 마지막 OFF 구간까지 포함해 총 3초

값은 `src/modules/realcar_calibration.h`의 `TIME_SYNC_*` 상수에서 바꾼다.
20 A는 검증값이 아니라 시작값이다. 반드시 구동륜을 지면에서 띄운 스탠드,
롤러 또는 통제된 시험 환경에서 HV Bus Current 식별성과 휠 가속을 확인한 뒤
조정한다.

## 시험 순서

1. Energy Meter와 Monolith 로깅을 먼저 시작한다.
2. 좌·우 컨트롤러 정상, TV/Regen/Paddock OFF, 스로틀 해제를 확인한다.
3. 시리얼 모니터에서 `SYNC_ARM`을 한 줄로 전송한다.
4. 진단 출력의 `SYNC arm=1`을 확인한다.
5. `SYNC_RUN`을 전송하고 `SYNC run=1`, `cmd=20.0` 파형을 확인한다.
6. 주행 종료 후 필요하면 같은 절차를 반복해 clock drift를 계산한다.

실차 바퀴가 지면에 닿아 있는 상태에서는 이 기능을 사용하지 않는다.
