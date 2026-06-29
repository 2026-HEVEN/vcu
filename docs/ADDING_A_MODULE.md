# 새 모듈 추가하기 (팀원용 가이드)

이 펌웨어는 **잠긴 코어(`src/core/`, `src/logic/`)** 와 **여러분이 채우는 모듈(`src/modules/`)** 로
나뉩니다. 여러분은 `src/modules/` 안의 `*_compute()` 함수만 작성합니다.
CAN·타이밍·안전은 코어가 알아서 합니다.

## 규칙 (꼭 지키기)
- 모듈 파일(`src/modules/*`)에서 `state.h` 와 `<Arduino.h>` 를 **include 하지 않는다.**
- 값은 `types.h` 의 도메인 타입(`Percent`, `Unit`, `Rpm` ...)으로 받고 돌려준다 — 범위가 자동으로 지켜진다.
- 계산에 이력(필터/적분)이 필요하면 **전역변수 대신 상태 struct를 인자로** 받는다 (`imu.h` 참고).

## 1. 모듈 만들기 (.h / .cpp 한 쌍)
`src/modules/coolant.h`
```cpp
#pragma once
#include "types.h"
struct CoolantInput  { int raw_adc; };
struct CoolantOutput { float temp_c; bool fan_on; };
CoolantOutput coolant_compute(const CoolantInput &in);   // 순수 함수
```
`src/modules/coolant.cpp`
```cpp
#include "modules/coolant.h"
CoolantOutput coolant_compute(const CoolantInput &in) {
    float t = in.raw_adc * 0.05f;          // TODO: 실제 변환식
    return { t, t > 80.0f };
}
```

## 2. 테스트 먼저/같이 쓰기 (하드웨어 없이 노트북에서)
`test/test_coolant/test_coolant.cpp`
```cpp
#include <unity.h>
#include "modules/coolant.h"
void test_fan_turns_on_when_hot(void) {
    TEST_ASSERT_TRUE(coolant_compute({2000}).fan_on);
}
void setUp(void){} void tearDown(void){}
int main(int,char**){ UNITY_BEGIN(); RUN_TEST(test_fan_turns_on_when_hot); return UNITY_END(); }
```
실행: `pio test -e native -f test_coolant`

## 3. 공유 상태에 필드 추가 (코어 담당자에게 요청 or 직접)
`include/state.h` 의 `VehicleState` 에 필요한 필드를 추가한다. 예: `bool fan_on;`

## 4. 배선 한 줄 + 스케줄 한 줄 (코어 담당자 영역)
`src/core/app_wiring.cpp`:
```cpp
static void coolant_update() {
    CoolantOutput o = coolant_compute({ analogRead(PIN_COOLANT_ADC) });
    state.fan_on = o.fan_on;
}
// g_tasks[] 배열에 한 줄 추가:
{ coolant_update, 100, 0 },   // 10 Hz
```

## 5. 설정·상태가 있는 센서(PCNT/SPI/I2C/타이머)는?
직접 `analogRead` 한 줄로 안 되는 페리페럴은 **잠긴 드라이버**를 `src/core/drivers/` 에 만들고
(예: `wss_driver`), 모듈의 `compute()` 는 그 드라이버가 준 깔끔한 원시값만 변환한다.
드라이버 인터페이스(.h)는 코어 담당자와 함께 정한다.

## 체크리스트
- [ ] `*_compute()` 가 `state.h`/`Arduino.h` 를 include 하지 않는다
- [ ] 입출력이 도메인 타입(`Percent` 등)이다
- [ ] `pio test -e native -f test_<name>` 통과
- [ ] `app_wiring.cpp` 에 `*_update()` + `g_tasks[]` 한 줄 추가
- [ ] `pio run -e esp32dev` 빌드 성공
