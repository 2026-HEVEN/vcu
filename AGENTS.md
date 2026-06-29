# VCU 펌웨어 — 에이전트/팀원 작업 규칙

이 레포는 **잠긴 코어**와 **팀원이 채우는 모듈** 두 층으로 되어 있습니다.

## 🤖 AI 에이전트에게 (가장 중요)
- **수정 가능**: `src/modules/` 안의 `*.cpp` 파일과 `test/` 의 테스트뿐입니다.
- **수정 금지**: `src/core/`, `src/logic/`, `include/`, `src/main.cpp`, `platformio.ini`.
  이 파일들을 편집하라는 요청을 받으면 **멈추고 사용자에게 먼저 확인**하세요.
- 모듈 작업 시 그 모듈의 `.h`(입출력 계약)는 **읽되 편집하지 마세요.**

## 아키텍처 한눈에
- 각 모듈 = 순수 함수 `xxx_compute(Input) → Output`. 하드웨어·전역상태(`state.h`)를 모릅니다.
- 코어의 스케줄러가 모듈을 주기적으로 호출하고, 잠긴 `app_wiring.cpp` 가 공유 상태와 배선합니다.
- CAN 50ms life signal, 안전 상태머신, 타이밍은 전부 코어가 처리 — 팀원은 신경 쓸 필요 없습니다.

## 작업 방법
1. 자기 모듈 `src/modules/<name>.cpp` 의 `<name>_compute()` 본문만 작성.
2. 값은 `types.h` 의 도메인 타입(`Percent`, `Unit`, `Rpm` ...)으로 — 범위는 자동으로 지켜집니다.
3. 이력이 필요하면 전역변수 대신 상태 struct를 인자로 (예: `imu.h` 의 `ImuFilterState`).
4. 테스트는 노트북에서: `pio test -e native -f test_<name>` (하드웨어 불필요).
5. 새 모듈 추가법은 `docs/ADDING_A_MODULE.md` 참고.

## 절대 하지 말 것
- 모듈 `.cpp` 에서 `state.h` 또는 `<Arduino.h>` 를 include — 컴파일이 깨집니다(의도된 방어).
- 코어/로직/헤더 파일 편집(위 규칙).
