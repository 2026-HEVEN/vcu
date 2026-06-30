# VCU 펌웨어 아키텍처

> 이 문서는 코드베이스 전체 구조를 처음 보는 사람이 이해할 수 있도록 설명합니다.
> 새 모듈을 추가하는 **실전 절차**는 [`ADDING_A_MODULE.md`](ADDING_A_MODULE.md)를 보세요.

## 0. 한 문장 요약

> **위험한 건 다 잠가두고(코어), 팀원은 입력→출력 수학 함수 하나(`compute`)만 채운다.
> 코어가 그 함수를 알아서 불러주고, CAN으로 내보내고, 안전하게 막아준다.**

---

## 1. 두 개의 층 — 전체의 뼈대

```
┌─ 🔒 LOCKED (코어 담당자만 수정) ──────────────────┐
│  include/      도메인 타입, CAN 프로토콜, 공유상태 정의 │
│  src/logic/    순수 로직 (스케줄러·안전 FSM·스케일링)   │
│  src/core/     하드웨어 (CAN·드라이버·배선·진입점)      │
└────────────────────────────────────────────────────┘
┌─ ✏️ FILL-IN (팀원이 채움) ─────────────────────────┐
│  src/modules/  xxx_compute() 7개                    │
└────────────────────────────────────────────────────┘
```

각 파일 맨 위 배너(`[LOCKED FILE]` / `[FILL-IN]`)가 사람과 AI 에이전트 모두에게 경계를 알립니다.
그리고 이 경계는 **말이 아니라 컴파일러로 강제**됩니다(§4, §10 참고).

---

## 2. 데이터 흐름

```
 [센서/드라이버]            [순수 모듈]                [공유 상태]            [CAN]
 throttle 핀 ──► throttle_compute   ──► state.throttle_pct ─┐
 brake 핀    ──► brake_compute      ──► state.brake_pct    ─┤
 엔코더 I2C  ──► steering_compute   ──► state.steering_angle┤
 MPU6050     ──► imu_compute        ──► state.yaw_rate      ├─► longitudinal_compute ─► state.total_torque
 휠 PCNT     ──► wheel_speed_compute ─► state.wheel_speed   ┘                                  │
                                                                                              ▼
                                                                torque_vectoring ─► state.torque_L / torque_R
                                                                                              │
                                                                [50ms 라이프 태스크] ─CAN─► 컨트롤러 L/R
```

- **센서 모듈 5개** (throttle/brake/steering/imu/wheel_speed): raw값 → 물리값 변환
- **`longitudinal`**: throttle/brake/SOC/모드 → **부호 있는 총 토크 1개** (+ 구동 / − 회생)
- **`torque_vectoring`**: 총 토크를 **좌우로 분배**
- 모든 모듈은 `state`(공유 버스)를 통해서만 데이터를 주고받음

---

## 3. 모듈의 두 겹 — `compute()` vs `update()` (가장 중요)

### (a) 팀원이 채우는 순수 함수 — `src/modules/throttle.cpp`

```cpp
Percent throttle_compute(const ThrottleInput &in) {   // 입력 구조체 → 출력 타입
    float frac = (float)in.raw_adc / 4095.0f;
    if (frac < 0.05f) return Percent(0.0f);            // 데드존
    return Percent((frac - 0.05f) / 0.95f * 100.0f);
}
```

하드웨어도, `state`도, `Arduino.h`도 **모른다.** 그냥 `int` 받아 `Percent` 돌려주는 수학.

### (b) 코어가 작성·잠근 배선 — `src/core/app_wiring.cpp`

```cpp
static void throttle_update() {
    state.throttle_pct = throttle_compute({ analogRead(PIN_THROTTLE_ADC) });
    //   └ 공유상태에 쓰기        └ 핀에서 raw 읽기 (코어만 함)
}
```

→ **팀원은 `compute()` 안쪽만, `analogRead`/`state`는 코어가 책임진다.**
이 분리 덕분에 노트북에서 하드웨어 없이 테스트가 된다(§10).

---

## 4. 공유 상태가 "전역인데 안전한" 이유

`VehicleState state;` 는 **단 한 파일** `app_wiring.cpp`에서만 정의·접근된다.
`state.h`(정의)는 `include/`에 있지만 — **모듈 `.cpp`는 `state.h`를 include하지 않는다.**

```cpp
// 팀원이 throttle.cpp에서 state를 만지려 하면?
state.torque_L = 999;   // ❌ 컴파일 에러 — state라는 이름 자체가 안 보임
```

"수정하지 마세요"라는 약속이 아니라 **구조적으로 불가능**하다.
누가 무엇을 읽고 쓸지는 `app_wiring.cpp`의 `*_update()` 배선이 100% 통제한다.

---

## 5. `Clamped` 타입 — 범위를 자료형에 박아넣기

`include/types.h`:

```cpp
template <int LO, int HI> class Clamped {
    float v_;
    void set(float x){ v_ = x<LO?LO : (x>HI?HI:x); }   // 대입할 때마다 잘림
public:
    Clamped(float x=0){ set(x); }
    Clamped& operator=(float x){ set(x); return *this; }
    operator float() const { return v_; }              // float처럼 읽힘
};
using Percent = Clamped<-100,100>;  // 스로틀·토크
using Unit    = Clamped<-1,1>;       // 스티어링
using Rpm     = Clamped<0,6000>;     // 속도
```

`Percent x = 300;` → **자동으로 100.** 범위 밖 값을 저장할 방법 자체가 없다.
모듈이 버그로 999를 내놔도 CAN으로 나가기 전에 타입이 막는 **2차 안전망**.
(C++17 제약상 정수 경계만 가능 — ±100, 0~100 등.)

---

## 6. 런타임 — 누가 언제 도나 (하이브리드)

진입점 `src/main.cpp`:

```cpp
void setup() {
    modules_init();              // 드라이버·CAN 초기화
    can_bus::start_life_task();  // ★별도 태스크 시작★
}
void loop() { scheduler_run(); } // 협력형 스케줄러만 반복
```

### (A) `loop()` 안 협력형 스케줄러 — `src/logic/scheduler_logic.cpp`

```cpp
void scheduler_tick(Task *tasks, int n, uint32_t now) {
    for (int i=0;i<n;i++)
        if (now - tasks[i].last_run >= tasks[i].period_ms) {
            tasks[i].update();          // 주기 됐으면 호출
            tasks[i].last_run = now;
        }
}
```

태스크 표(`app_wiring.cpp`)가 "무엇을 얼마나 자주" 정한다:

```cpp
Task g_tasks[] = {
    { throttle_update,         10, 0 },  // 100Hz
    { torque_vectoring_update, 10, 0 },  // 100Hz
    { can_rx_update,            5, 0 },  // 200Hz
    { debug_update,           200, 0 },  // 5Hz
};
```

→ **새 모듈 추가 = 이 표에 한 줄.** 팀원이 코어에서 건드리는 유일한 접점.

### (B) 별도 고우선순위 FreeRTOS 태스크 — 50ms 라이프 신호 — `src/core/can_bus.cpp`

```cpp
void life_task(void *) {
    TickType_t next = xTaskGetTickCount();
    for (;;) {
        bool allow = torque_allowed() && (millis()-g_last_cmd_ms < 200);
        float l = allow ? (float)state.torque_L : 0.0f;   // ★안전: 못 가면 0★
        float r = allow ? (float)state.torque_R : 0.0f;
        send_torque(CAN_ID_TORQUE_L, l);
        send_torque(CAN_ID_TORQUE_R, r);
        g_life++;
        vTaskDelayUntil(&next, pdMS_TO_TICKS(50));         // 정확히 50ms 간격
    }
}
```

**왜 핵심인가:** 규정상 50ms마다 라이프 신호를 못 보내면 컨트롤러가 HALT된다.
이걸 `loop()` 안에 넣으면 누군가의 `compute()`가 한 번 느려질 때(I2C 블로킹 등)
라이프 신호가 밀려 **주행 중 차가 멈출 수 있다.** 그래서 다른 코어(core 1)에 고정된
전용 태스크로 빼서, `loop()`이 아무리 막혀도 50ms는 절대 안 밀린다.
**"잠긴 코어 / 채우는 모듈"의 존재 이유가 바로 이 안전 보장이다.**

---

## 7. 안전 상태머신 — `src/logic/safety_logic.cpp`

```cpp
SafetyState safety_step(SafetyState cur, const SafetyInputs &in) {
    if (!in.shutdown_ok) return SafetyState::Halt;       // 셧다운 끊기면 즉시 HALT
    switch (cur) {
        case Idle:  return in.handshaked    ? Ready : Idle;
        case Ready: return in.start_pressed ? Drive : Ready;
        case Drive: return in.deadman_ok    ? Drive : Halt;  // 명령 끊기면 HALT
        case Halt:  return Halt;                             // 래치 (전원 재투입 전까지)
    }
}
```

토크는 `torque_allowed()`(= 상태가 `Drive`일 때만 true)가 통과해야 나간다.
순수 함수라 노트북에서 모든 전이를 테스트했다.
`src/core/safety.cpp`(코어)는 GPIO를 읽어 이 함수에 넣어주는 껍데기.

---

## 8. 페리페럴 드라이버 패턴 — 설정이 필요한 센서

`analogRead` 한 줄이면 되는 throttle과 달리, **PCNT(휠속)·I2C(IMU)·SPI/I2C(엔코더)**는
설정·상태가 있어 `src/core/drivers/`에 **잠긴 얇은 드라이버**로 분리한다.

```
wss_driver (PCNT 설정·카운트) ──► {pulse_delta, dt_ms} ──► wheel_speed_compute ──► Rpm
   [LOCKED 하드웨어]                    깔끔한 raw            [FILL-IN 순수 수학]
```

팀원은 `wheel_speed_compute`(펄스→속도, 캘리브레이션)만 손대고 PCNT 레지스터는 안 본다.
IMU·엔코더도 동일 구조.

> 참고: PCNT 드라이버는 Arduino-ESP32가 ESP-IDF 4.4.7을 쓰는 관계로
> v5 API(`driver/pulse_cnt.h`) → v4 API(`driver/pcnt.h`)로 포팅돼 있다. 동작은 동일.

---

## 9. 상태가 필요한 변환 — IMU 필터

필터·적분처럼 **이전 값이 필요한** 계산은 전역변수 대신 **상태 구조체를 인자로** 넘긴다.

```cpp
ImuOutput imu_compute(const ImuRaw &raw, ImuFilterState &s);  // s에 이전값 보관·갱신
```

`app_wiring`이 `static ImuFilterState imu_state;`를 들고 매번 넘겨준다.
덕분에 테스트에서 초기 상태를 직접 주입해 **결정론적으로** 검증 가능
(전역 static이면 테스트가 서로 오염됨).

---

## 10. 테스트 — 노트북에서 하드웨어 없이

`platformio.ini`에 두 환경:

| 환경 | 용도 | 명령 |
|------|------|------|
| `esp32dev` | 실제 펌웨어 빌드/업로드 | `pio run -e esp32dev` |
| `native` | 노트북에서 순수 코드 단위 테스트 | `pio test -e native` |

`native`는 `build_src_filter`로 `src/modules/` + `src/logic/`만 컴파일한다.
그래서 `src/core/`(하드웨어)는 아예 안 보이고 `compute()` 함수들만 Unity로 검증한다.

이것이 **모듈이 `state.h`/`Arduino.h`를 include하면 안 되는 이유**이기도 하다 —
그러면 native 빌드가 깨지므로, 순수성이 컴파일러로 강제된다.

> 현재 기준 native 테스트 **38개 전부 통과**, ESP32 빌드 그린.

---

## 11. 팀원의 전체 작업 사이클

1. `src/modules/<name>.cpp`의 `<name>_compute()` 본문 작성
2. `test/`에 테스트 쓰고 `pio test -e native -f test_<name>` 로 확인
3. (코어 담당) `app_wiring.cpp`에 `*_update()` + `g_tasks[]` 한 줄 추가
4. `pio run -e esp32dev` 로 빌드 → 보드에 업로드
5. `pio device monitor` 로 `debug_monitor`(5Hz)가 뿌리는 state 실시간 확인

상세 절차: [`ADDING_A_MODULE.md`](ADDING_A_MODULE.md)

---

## 핵심 요약

| 장치 | 역할 |
|------|------|
| **2층 구조 + 배너** | 위험/안전 코드 격리, 사람·AI에게 경계 표시 |
| **compute / update 분리** | 팀원은 수학만, 코어가 하드웨어·배선 |
| **state include 차단** | "남의 출력 못 건드림"을 컴파일러가 강제 |
| **Clamped 타입** | 범위 밖 값 저장 불가 (2차 안전망) |
| **별도 50ms 라이프 태스크** | loop가 막혀도 차 안 멈춤 (1차 안전 보장) |
| **안전 FSM + 토크 게이트** | Drive 상태 아니면 토크 0 |
| **native 테스트 환경** | 하드웨어 없이 노트북에서 검증 |

## 알려진 미완성 (의도된 TODO)

- `can_bus.cpp::poll_rx()` — CAN RX 파싱/핸드셰이크 미구현 → 현재 FSM이 `Idle`에 머묾.
  실제 컨트롤러 연동 시 구현 필요.
- `longitudinal_compute` / `tv_compute` — 안전한 stub(선형 매핑·50:50 분배).
  토크벡터링팀이 실제 전략으로 채울 FILL-IN 지점.
- deadman은 현재 "스케줄러 살아있음"을 추적(자세한 NOTE는 `can_bus.cpp` 참고).
  RX/handshake 구현 후 "명령 신선도"로 게이팅 예정.
