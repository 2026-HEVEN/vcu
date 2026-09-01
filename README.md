# HEVEN VCU 펌웨어

**2026 영광 대회 · 차량 제어 유닛(Vehicle Control Unit)** — ESP32가 스로틀/브레이크/조향/IMU/휠속을 읽어 토크를 계산하고, CAN으로 좌우 모터컨트롤러(EZkontrol)를 구동합니다. 안전 임계 보드입니다.

> 🤖 **AI 에이전트/팀원은 [`AGENTS.md`](AGENTS.md)를 먼저 읽으세요** — 무엇을 고쳐도 되고 무엇을 건드리면 안 되는지 규칙이 있습니다.

---

## 한눈에

- **스택**: PlatformIO + Arduino-ESP32, ESP32 내장 TWAI(CAN)
- **구조**: 잠긴 코어(CAN·안전·타이밍) + 팀원이 채우는 순수 모듈(`src/modules/`). 자세히는 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)
- **상태**: ESP32 빌드 그린, 호스트 테스트 121개 통과

## 빠른 시작

```bash
# 1. 이 레포를 깨끗한 위치에 클론
git clone https://github.com/2026-HEVEN/vcu.git
cd vcu

# 2. PlatformIO 설치 (한 번만) — VS Code면 PlatformIO IDE 확장 설치로 대체 가능
uv tool install platformio      # 또는: pipx install platformio

# 3. 노트북에서 테스트 (하드웨어 불필요)
pio test -e native

# 4. 보드에 빌드 & 업로드 (ESP32 연결 상태에서)
pio run -e esp32dev -t upload

# 5. 실시간 상태 모니터 (throttle/torque 등 5Hz 출력)
pio device monitor
```

> 🧪 **테스트가 처음이거나 Windows 사용자라면** → 노션 [펌웨어 테스트 실행 방법](https://www.notion.so/390913e532e68199a9b5e340b73e9e71) 참고. AI에 복붙할 프롬프트 + "이렇게 나오면 성공" 출력 예시 + Windows(WSL2/MinGW) 셋업까지 있습니다. (보드 업로드는 Windows도 그냥 되고, `native` 단위테스트만 host 컴파일러가 필요해요.)

## 어디서 작업하나

| 폴더 | 내용 | 편집? |
|------|------|-------|
| `src/modules/` | 순수 계산 함수 `xxx_compute()` | ✅ **여기서만** |
| `test/` | 노트북 단위 테스트 | ✅ |
| `src/core/`, `src/logic/`, `include/` | CAN·안전·스케줄러·드라이버·타입 | 🔒 잠김 |
| `platformio.ini`, `src/main.cpp` | 빌드 설정·진입점 | 🔒 잠김 |

새 모듈을 추가하거나 기존 `compute()`를 채우는 법 → [`docs/ADDING_A_MODULE.md`](docs/ADDING_A_MODULE.md)

## 모듈 목록 (FILL-IN)

`throttle` · `brake` · `steering` · `imu` · `wheel_speed` · `vehicle_speed`(4륜→차속) · `longitudinal`(회생 전략) · `torque_vectoring`(좌우 분배)
— 각 모듈은 `xxx_compute(입력) → 출력` 순수 함수. 하드웨어·전역상태를 모릅니다.

## 문서

| 문서 | 내용 |
|------|------|
| [`AGENTS.md`](AGENTS.md) / `CLAUDE.md` | 작업 규칙 (에이전트·팀원 필독) |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | 전체 설계 해설 (2층 구조·안전·테스트) |
| [`docs/ADDING_A_MODULE.md`](docs/ADDING_A_MODULE.md) | 모듈 추가/작성 절차 |
| [`docs/CAN_PROTOCOL.md`](docs/CAN_PROTOCOL.md) | CAN 메시지 명세 (VCU/Cluster 공유 단일 출처) |
| [`docs/TORQUE_VECTORING.md`](docs/TORQUE_VECTORING.md) | 5-stage 토크벡터링 설계와 안전 동작 |
| [`docs/REALCAR_CALIBRATION.md`](docs/REALCAR_CALIBRATION.md) | 실차 확정값·초기값·측정/튜닝 절차 |
| [`src/core/board_pins.h`](src/core/board_pins.h) | 하네스 v5 GPIO 단일 기준 (`GPIO-fixed` 동기화) |

## 현재 bring-up 범위와 남은 작업

- **구동 경로 구현 완료** — 좌/우 EZkontrol 핸드셰이크, 50 ms life frame,
  목표속도/RUNNING bit, deadman, 스로틀-release arm 조건이 연결되어 있다.
- **컨트롤러 피드백 파싱 미구현** — 실제 전류·RPM·온도·오류 프레임을 아직
  `VehicleState`와 진단 CAN으로 전달하지 않는다. 최초 시험은 CAN analyzer와
  컨트롤러/BMS 로그를 함께 사용한다.
- **현재 프로파일은 2모터 저속 시험용** — 브레이크 센서는 비활성화되어 있지만,
  좌·우 컨트롤러가 모두 핸드셰이크되어야 arm 가능하다. 브레이크 센서 장착 후
  `realcar_calibration.h`의 센서 플래그를 생산 조건으로 되돌린다.
- **토크벡터링 실차 활성화** — 5-stage 로직은 구현되어 있지만 기본 PID는
  `0/0/0`이라 strict 50:50 OFF다. 실차 캘리브레이션과 단계별 검증 후 활성화한다.

> ⚠️ 안전: HV·토크가 걸리는 보드입니다. 첫 스탠드 테스트 전 잠긴 코어(특히 `safety`·`can_bus`)를 임의로 수정하지 마세요.

## 버전 기록 (Changelog)

> 각 버전은 git 태그로도 관리됩니다 → [GitHub Releases](https://github.com/2026-HEVEN/vcu/releases)
> **새 버전 올릴 때:** 아래에 항목 추가 → `git tag vX.Y` → `git push origin vX.Y`.

### v1.1 (2026-06-29) — CAN 프로토콜 갱신
- Cluster→VCU 커맨드(`0x1801D0C0`) 레이아웃(`CAN_PROTOCOL.md` §5.7)을 **gear · drive_mode · paddock** 로 확정 (Cluster 재설계와 동기화)

### v1.0 (2026-06-29) — VCU 펌웨어 베이스
- 잠긴 코어(TWAI + **50ms 라이프 태스크** + 안전 FSM + 협력형 스케줄러) + 순수 모듈 8개(throttle · brake · steering · imu · wheel_speed · vehicle_speed · longitudinal · torque_vectoring)
- `Clamped<>` 도메인 타입, PlatformIO native 테스트, 5Hz 시리얼 디버그 모니터
- CAN 프로토콜 단일 출처(토크 스케일링 + 피드백/커맨드 ID + 디코더), ARCHITECTURE/ADDING_A_MODULE/CAN_PROTOCOL 문서, LOCKED/FILL-IN 배너
