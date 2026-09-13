# M2 — 좌우 모터 명령 스냅샷과 크로스코어 원자성

작업 브랜치: `fix/m2-command-snapshot` (base `origin/dev` @ `8498e2f`)
상태: 설계 확정, 구현 대기
대상 결함: 코드 리뷰 MEDIUM · M2 (지적 ③)

이 문서는 사람과 AI 에이전트 모두를 대상으로 한다. 이 작업을 이어받는
에이전트는 이 문서의 "검증된 사실"과 "수정 범위"를 그대로 신뢰해도 되지만,
줄 번호는 `origin/dev` @ `8498e2f` 기준이므로 편집 전 반드시 재확인한다.

---

## 1. 한 줄 요약

좌·우 모터 명령이 하나의 결정이 아니라 서로 다른 시점에 따로 읽히고 따로
송신되는 개별 값이다. 제어 로직이 좌우 균형을 맞춰 계산해도 전달 과정에서
그 균형이 깨질 수 있다. 최종 명령을 하나의 구조체로 원자 게시·복사하고,
안전 판정 이후에 발행하도록 순서를 고정해서 해결한다.

---

## 2. 현재 구조 (검증됨)

두 개의 실행 컨텍스트가 서로 다른 코어에서 동시에 돈다.

| 컨텍스트 | 코어 | 주기 | 역할 |
|---|---|---|---|
| `loop()` / `scheduler_run()` | core 0 | 10 ms (100 Hz) | 센서, 제어, 안전 판정. `state`를 쓴다 |
| `can_life` 태스크 | core 1 | 50 ms (20 Hz) | `state`를 읽어 EZkontrol 명령 프레임 송신 |

`can_life`는 `xTaskCreatePinnedToCore(life_task, "can_life", 4096, nullptr, 20, nullptr, 1)`로
core 1에 고정된다 (`src/core/can_bus.cpp:280`). 두 코어 사이에 락, 배리어,
큐 같은 동기화 장치가 명령 경로에 전혀 없다.

`src/core/can_bus.cpp:28-31`의 기존 주석은 "32비트 정렬 float 접근은
word-atomic이므로 한 사이클 stale 값은 무해하다"고 적고 있다. 이 진술 자체는
맞다. 문제는 그것이 **필드 단위 보장**이라는 점이다. 필요한 것은
`torque_L`, `torque_R`, `gear`, 안전 판정을 **묶어서** 같은 순간의 값이라는
보장이며, 그것은 어디에도 없다.

---

## 3. 결함 상세

### 3.1 좌·우가 서로 다른 tick에서 읽힌다

```
src/core/can_bus.cpp:164   float l = normal_allow ? (float)state.torque_L : 0.0f;
src/core/can_bus.cpp:165   float r = normal_allow ? (float)state.torque_R : 0.0f;
src/core/app_wiring.cpp:328   state.torque_L = out.left_a;
src/core/app_wiring.cpp:329   state.torque_R = out.right_a;
```

core 1이 164행과 165행 사이에 있을 때 core 0이 328-329행을 실행하면
`l`은 tick N, `r`은 tick N+1의 값이 된다.

심각도는 tick 간 변화량이 결정한다.

- **상승 방향**: `limit_rising_magnitude()` (`src/modules/drive_supervisor.cpp:44-58`)가
  `max_step = drive_current_max_per_motor_a * dt / drive_current_rise_time_s`
  = `500 * 0.01 / 0.5` = 10 A/tick으로 제한한다. 좌우 불일치 최대 10 A. 무해.
- **하강 방향**: 슬루 제한이 없다. 주석
  (`src/modules/realcar_calibration.h:50-52`)이 "release and protection cuts
  remain immediate"라고 명시한다. 한 tick에 전체 명령이 0으로 떨어질 수 있고,
  그 순간 찢어지면 좌우 불일치 = 명령 전체.

### 3.2 `send_torque()`가 공유 `gear`를 직접, 두 번 읽는다

```
src/core/can_bus.cpp:99-104
    const int target_rpm = !running ? 0
        : (state.gear == Gear::Reverse ? -DRIVE_TARGET_SPEED_RPM
            : (amps < 0.0f ? REGEN_TARGET_SPEED_RPM
                           : DRIVE_TARGET_SPEED_RPM));
src/core/can_bus.cpp:255   if (g_handshaked_L) send_torque(CAN_ID_TORQUE_L, l, run_l);
src/core/can_bus.cpp:256   if (g_handshaked_R) send_torque(CAN_ID_TORQUE_R, r, run_r);
```

`send_torque()`는 `gear`를 인자로 받지 않고 전역 `state`에서 직접 읽는다.
이 함수는 한 주기에 두 번 호출되므로 `gear`를 두 번 따로 읽는다.

경쟁 창이 3.1보다 훨씬 넓다. `send_torque()` 내부의
`twai_transmit(&m, pdMS_TO_TICKS(5))`가 TX 큐가 찼을 때 최대 5 ms 블록한다.
즉 두 번의 `gear` 읽기가 나노초가 아니라 **최대 5 ms** 떨어져 있다.
core 0의 제어 주기가 10 ms이므로 그 사이에 `gear`가 갱신될 수 있다.

결과는 크기 차이가 아니라 **방향 차이**다. 왼쪽 프레임에 `+4000 rpm`,
오른쪽 프레임에 `-4000 rpm`이 실릴 수 있다.

추가로, 이 경로는 방향 인터록을 **우회한다**. `direction_interlock_update()`
(`src/core/app_wiring.cpp:228-237`)는 정지와 스로틀 해제를 확인한 뒤에야
방향을 arm 하고, 그 결과 `command_sign`을 `state.total_torque`의 **부호**에만
적용한다. 반면 CAN 프레임의 목표 rpm 부호는 인터록의 armed 방향이 아니라
**원시 `state.gear`**에서 온다. 게다가 그 읽기는 게이트(`can_bus.cpp:157-163`)
**이후**에 일어난다. 따라서 게이트 통과 시점에는 Drive였다가 5 ms 뒤
Reverse로 바뀌면, `run = true`와 구동 전류를 실은 채 목표 rpm만 반대 부호로
나갈 수 있다. 인터록은 이 창을 막지 못한다.

EZkontrol이 토크 제어 모드에서 반대 부호 목표 rpm을 받았을 때의 실제 거동은
**미검증**이다(리뷰 H3 항목). 가능한 결과는 "무해한 클램프"부터 "한쪽은 구동,
반대쪽은 제동"까지 폭이 넓다. 검증되지 않았다는 사실 자체가 보내지 말아야 할
이유다.

같은 주기 안에서 `gear`를 총 세 번 읽는다. 세 읽기가 같은 값이라는 보장이 없다.

| 위치 | 용도 |
|---|---|
| `can_bus.cpp:157` | `propulsion_gear` (구동 기어인지) |
| `can_bus.cpp:101` (L 호출) | 왼쪽 목표 rpm 부호 |
| `can_bus.cpp:101` (R 호출) | 오른쪽 목표 rpm 부호 |

### 3.3 한쪽 송신 실패가 무시된다

`send_torque()`는 `twai_transmit()`의 반환값을 받지 않는다
(`src/core/can_bus.cpp:110`). 같은 파일의 `transmit_ext()`는 `bool`을
반환하지만 `send_torque()`는 별도 경로다.

결과:

- L은 큐에 들어가고 R이 실패하면, 오른쪽 컨트롤러는 이번 주기 프레임을 못 받고
  직전 명령을 유지한다. 좌우 비대칭.
- `state.can_commanded_current_L/R` (`can_bus.cpp:251-254`)에는 실패와 무관하게
  기록된다. 로그가 사실과 다르다.
- 실패 카운터, 연속 실패 정책, 진단 출력이 전혀 없다.

버스 부하는 낮으므로 (250 kbit/s, 주기당 모터 2프레임 + 20 Hz 텔레메트리)
큐 포화보다는 error-passive, ACK 없음, bus-off 직전 상태가 현실적인 유발 원인이다.

### 3.4 안전 판정과 명령 값의 시점이 분리되어 있다

`src/core/app_wiring.cpp:361-363`의 태스크 순서:

```
{ torque_vectoring_update, 10, 0 },
{ time_sync_pulse_update,  10, 0 },
{ drive_supervisor_update, 10, 0 },   <- state.torque_L/R 를 쓴다
{ safety_task,             10, 0 },   <- 안전 FSM은 그 다음에 돈다
```

따라서 `state.torque_L/R`에 들어 있는 값은 **그 tick의 안전 판정을 거치지 않은**
값이다. 유일한 최종 게이트는 `life_task`가 부르는 `torque_allowed()`
(`can_bus.cpp:158`)이며, 이 호출과 값 읽기(164-165행)는 분리되어 있다.

두 가지 결과:

1. 게이트를 통과시킨 판정과 실제로 실린 값이 서로 다른 순간의 것이다.
   최대 한 발행 주기(50 ms) 동안 HALT 이전 명령이 나갈 수 있다.
2. 사후 추적이 불가능하다. 로그만으로 "이 명령은 이 판정에 근거해 나갔다"를
   말할 수 없다.

### 3.5 최종 명령 경로에 NaN/Inf 검사가 없다 (추가 발견)

`Clamped<LO,HI>::set()` (`include/types.h:13`)은

```cpp
void set(float x) { v_ = x < LO ? (float)LO : (x > HI ? (float)HI : x); }
```

NaN과의 비교는 모두 false이므로 **NaN이 그대로 통과한다**. `Amp`는 NaN을
막지 못한다.

`drive_supervisor.cpp`에는 `std::isfinite` 검사가 하나도 없다. TV 각 스테이지는
입력을 검사하지만(`tv/allocation.cpp:12`, `tv/load.cpp:10`,
`tv/reference.cpp:14`, `tv/traction.cpp:13`, `torque_vectoring.cpp:20,47`),
`drive_supervisor` → `state.torque_L/R` → CAN 프레임 구간에는 검사가 없다.

노션 "반드시 남겨야 할 로직"에 `명령값의 NaN/Inf 검사`가 올라 있으나,
실제로는 최종 단계에 존재하지 않는다. M2에서 같이 넣는다.

---

## 4. 위험도

한쪽 모터만 전류를 받고 반대쪽이 0일 때 발생하는 요 모멘트의 이론 상한.
계산 재료는 전부 `src/modules/realcar_calibration.h`에 있다.

```
500 A × 0.1266 Nm/A  = 63.3 Nm    모터축 토크      MOTOR_KT_NM_PER_A          :148
63.3 Nm × 3.72       = 235.5 Nm   휠축 토크        GEAR_RATIO                 :147
235.5 Nm ÷ 0.2387 m  = 987 N      타이어 접지 추력  WHEEL_SPEED_ROLLING_RADIUS_M :162
987 N × (1.090/2) m  = 538 Nm     요 모멘트        REAR_TRACK_M               :172
```

이 값은 상한이며 실제로는 두 가지가 깎는다.

- 타이어가 987 N을 전달하지 못하면 휠스핀으로 빠져 실제 요 모멘트는 더 작다.
- 10 kW 규정 전력 제한이 대부분의 속도에서 500 A보다 먼저 걸린다.
  36 km/h 부근에서 10 kW면 총 추력 약 1000 N, 한쪽 몰림 시 약 500 N,
  요 모멘트 약 270 Nm.

발생 조건이 좋지 않다. 좌우 불일치가 최대가 되는 순간은 **구동 전류를 0으로
끊는 순간**이다(페달 해제, 브레이크, 피드백 stale/fault, 안전 HALT, 핸드셰이크
손실). 이 순간은 제동 하중 이동으로 리어 그립이 가장 낮은 시점과 겹친다.

발생 확률은 낮다. 재현이 되지 않고 로그에도 남지 않는 종류의 결함이므로,
확률이 아니라 결과와 추적 불가능성으로 우선순위를 매긴다.

---

## 5. 수정 설계

핵심 아이디어 하나다. **최종 명령에 필요한 모든 값을 하나의 구조체로 묶어,
core 0이 원자적으로 게시하고 core 1이 원자적으로 통째 복사한다.**
이후 core 1은 복사본만 본다. 3.1, 3.2, 3.4가 같은 수정으로 사라진다.

### 5.1 새 순수 모듈 `src/modules/motor_command.{h,cpp}`

의사결정 로직 전부를 하드웨어와 전역상태를 모르는 순수 함수로 옮긴다.
`platformio.ini`의 `[env:native]`가 `build_src_filter = +<modules/> +<logic/>`
이므로 노트북에서 단위 테스트가 가능하다.

```cpp
#pragma once
#include <cstdint>
#include "modules/gear.h"   // enum class Gear

// core 0이 제어 tick마다 한 번 게시하고, core 1이 통째로 복사한다.
struct MotorCommandSnapshot {
    uint32_t seq;                        // 게시마다 1 증가
    uint32_t published_ms;               // 게시 시각
    float    left_a;
    float    right_a;
    Gear     gear;
    bool     safety_allow;               // 같은 tick의 torque_allowed()
    bool     throttle_signal_valid;
    bool     propulsion_direction_armed;
    bool     brake_active;
    bool     controller_feedback_fresh;
    bool     controller_fault_latched;
};

// core 1이 자기 소유로 관리하는 게이트. 공유 상태가 아니다.
struct MotorCommandGates {
    bool  scheduler_alive;
    bool  reconnect_inhibit;
    bool  component_test_inhibit;
    bool  snapshot_fresh;
    float reconnect_ramp_scale;          // 램프 없으면 1.0
};

struct MotorCommandParams {
    int drive_target_speed_rpm;          // 4000
    int regen_target_speed_rpm;          // 0
};

struct MotorFrameCommand {
    float left_a,  right_a;
    int   target_rpm_L, target_rpm_R;
    bool  run_L, run_R;
    bool  normal_allow;
};

MotorFrameCommand motor_command_resolve(const MotorCommandSnapshot &s,
                                        const MotorCommandGates &g,
                                        const MotorCommandParams &p);
```

`motor_command_resolve()`의 동작은 현재 `life_task`의 정상 경로와 **동일한
결과**를 내되, `gear`를 단 한 번만, 복사본에서 읽는다.

```
finite   = isfinite(s.left_a) && isfinite(s.right_a)
allow    = s.safety_allow && g.scheduler_alive && g.snapshot_fresh && finite
        && s.throttle_signal_valid && !g.reconnect_inhibit
        && !g.component_test_inhibit && s.propulsion_direction_armed
        && (s.gear == Gear::Drive || s.gear == Gear::Reverse)

left_a   = allow ? s.left_a  * g.reconnect_ramp_scale : 0
right_a  = allow ? s.right_a * g.reconnect_ramp_scale : 0
run_L    = run_R = allow

target_rpm(amps) = !allow             ? 0
                 : s.gear == Reverse  ? -p.drive_target_speed_rpm
                 : amps < 0           ?  p.regen_target_speed_rpm
                                      :  p.drive_target_speed_rpm
```

NaN 처리는 **양쪽 동시 차단**이다. 한쪽만 0으로 만들면 그 자체가 좌우 비대칭을
만든다. 노션의 "한쪽 컨트롤러 fault 시 양쪽 명령 0 A" 철학과 같다.

참고로 `tv_alloc_compute()` (`src/modules/tv/allocation.cpp:31-37`)는
`total_current_a`의 부호에 따라 좌우를 `[0, max]` 또는 `[-max, 0]`으로
클램프하므로, 좌우가 서로 반대 부호가 되는 일은 없다. 그럼에도 목표 rpm은
**측별로** 계산한다. 부호 일치에 의존하지 않기 위해서다.

### 5.2 원자 게시·복사 (LOCKED · `can_bus`)

```cpp
// can_bus.h
namespace can_bus {
    void publish_motor_command(const MotorCommandSnapshot &s);
}
```

```cpp
// can_bus.cpp (익명 네임스페이스)
portMUX_TYPE g_cmd_mux = portMUX_INITIALIZER_UNLOCKED;
MotorCommandSnapshot g_cmd_snapshot{};   // g_cmd_mux로 보호

MotorCommandSnapshot copy_motor_command() {
    MotorCommandSnapshot out;
    portENTER_CRITICAL(&g_cmd_mux);
    out = g_cmd_snapshot;
    portEXIT_CRITICAL(&g_cmd_mux);
    return out;
}
```

`portENTER_CRITICAL(portMUX_TYPE*)`는 ESP32에서 스핀락과 인터럽트 차단을 함께
수행하며 **코어 간 상호배제**를 보장한다. 임계구역 안에서는 구조체 복사만 한다.
약 40 byte 복사이므로 수십 ns이고, core 0에서 100 Hz, core 1에서 20 Hz로
진입한다. 인터럽트 차단 시간은 무시할 수준이다.

**CAN 송신은 반드시 임계구역 밖에서 한다.** `twai_transmit()`은 최대 5 ms
블록할 수 있으므로 임계구역 안에 두면 시스템이 멈춘다.

seqlock 같은 락프리 방식이 이론적으로는 더 우아하지만 메모리 배리어를 한 군데
빠뜨리면 조용히 깨지고 팀원이 읽기도 어렵다. 이 주기와 데이터 크기에서는
임계구역이 옳은 선택이다.

### 5.3 발행 순서 고정 (LOCKED · `app_wiring`)

태스크 테이블에서 `safety_task`를 `drive_supervisor_update` **앞**으로 옮긴다.

```
{ time_sync_pulse_update,  10, 0 },
{ safety_task,             10, 0 },   // 이동: 판정이 발행보다 먼저
{ drive_supervisor_update, 10, 0 },   // 끝에서 스냅샷 게시
```

`drive_supervisor_update()` 끝, `state.torque_L/R` 대입 직후에 게시한다.
게시하는 `safety_allow`는 **같은 tick에 갓 갱신된** `torque_allowed()`다.

`life_task`는 `torque_allowed()`를 더 이상 직접 부르지 않는다. 스냅샷 안의
`safety_allow`만 본다.

부작용 검증:

| 항목 | 영향 | 판단 |
|---|---|---|
| `safety_update()`가 읽는 `deadman_ok()` | `note_command()`가 `drive_supervisor_update` 끝에 있으므로 10 ms 오래된 타임스탬프를 본다 | 판정 창이 `DEADMAN_MS = 200`이므로 무해 |
| `safety_update()`가 읽는 `throttle_signal_valid`, `throttle_pct` | `throttle_update`가 테이블 2번째라 이동 후에도 앞선다 | 영향 없음 |
| `time_sync_pulse_update()`의 `torque_allowed()` (`app_wiring.cpp:263`) | 이동 후에도 `safety_task`보다 앞이므로 직전 tick 판정을 본다 | 현재와 동일. 변화 없음 |
| `send_sensor_telemetry()`의 `torque_allowed()` (`can_bus.cpp:325`) | core 0, 표시 전용 | 영향 없음 |
| `debug_update()`의 `torque_allowed()` (`debug_monitor.cpp:225`) | core 0, 표시 전용 | 영향 없음 |

### 5.4 송신 함수에서 전역 상태 접근 제거 (LOCKED · `can_bus`)

```cpp
bool send_torque(uint32_t id, float amps, int target_rpm,
                 bool running, uint8_t life);
```

- `state.`로 시작하는 접근이 함수 안에 하나도 남지 않아야 한다. 3.2의 직접 해결.
- `twai_transmit()`의 결과를 그대로 반환한다.

### 5.5 송신 실패 처리 (LOCKED · `can_bus`)

`life_task`에서:

```
ok_l = !g_handshaked_L || send_torque(CAN_ID_TORQUE_L, cmd.left_a,
                                      cmd.target_rpm_L, cmd.run_L, g_life);
ok_r = !g_handshaked_R || send_torque(CAN_ID_TORQUE_R, ...);

실패하면 해당 측 연속 실패 카운터 증가, 성공하면 0으로 리셋.
연속 실패가 MOTOR_TX_FAIL_LIMIT 이상이면
    invalidate_controller_link(측, "tx fail")
→ 기존 재연결 경로를 그대로 탄다. g_reconnect_inhibit → 양쪽 0 A →
  복구 시 0→1 램프.
```

이것이 노션의 "한쪽 컨트롤러 fault 또는 연결 단절 시 양쪽 명령 0 A"와
"재연결 시 토크를 0에서 점진적으로 복원"에 그대로 맞는다. 새 정책을
만들지 않고 기존 정책에 연결하는 것이 핵심이다.

**`MOTOR_TX_FAIL_LIMIT`은 3으로 한다.** 근거는 EZkontrol 자체 규칙이다
(`docs/CAN_PROTOCOL.md:261-266`).

| 신호 | 판정까지 | 결과 | 통제 주체 |
|---|---|---|---|
| VCU TX 연속 실패 3회 (신규) | 150 ms | 양쪽 0 A | VCU |
| EZkontrol life signal 5회 연속 실패 | 250 ms | MCU 자체 셧다운 후 재핸드셰이크 | 컨트롤러 |
| 컨트롤러 피드백 stale (`CONTROLLER_FEEDBACK_STALE_MS`) | 250 ms | 양쪽 0 A (기존) | VCU |
| EZkontrol 제어명령 10회 연속 실패 | 500 ms | MCU 셧다운 | 컨트롤러 |

한쪽에 프레임이 도달하지 못하면 **그 컨트롤러는 250 ms 뒤 스스로 셧다운한다.**
VCU가 무엇을 하든 그렇다. 따라서 "양쪽을 계속 굴린다"는 선택지는 존재하지 않는다.
VCU가 150 ms에 대칭적으로 끊지 않으면, 250 ms에 한쪽만 꺼지는 비대칭이
VCU 통제 밖에서 발생한다. 3회는 컨트롤러보다 먼저 움직이기 위한 값이다.

**컷은 즉시여야 한다.** 도달 불가능한 쪽은 자체 타임아웃까지 직전 명령을
유지하므로, 도달 가능한 쪽만 완만하게 내리면 그 시간만큼 좌우 차이가 유지된다.
즉시 컷이 비대칭 창을 가장 짧게 만든다.

**3은 잠정치다.** 한쪽만 실패할 때 비대칭 지속 시간은 `250 ms - 컷 시점`이라
늦게 끊을수록 짧다. 다만 이 계산은 "프레임을 놓친 EZkontrol이 자체 타임아웃
전까지 직전 명령을 유지한다"는 미검증 가정 위에 있고, 첫 누락에서 바로 0으로
간다면 결론이 반대가 된다. 확정 절차는 `docs/M2_FOLLOWUP_ITEMS.md` 1번에 있다.

**M2의 블로커는 아니다.** 이 상수는 `realcar_calibration.h`에서 언제든 바꾼다.
`twai_transmit()` 실패는 로컬 조건이고 TWAI 주변장치와 버스를 양쪽 ID가
공유하므로 한쪽만 실패하는 경우 자체가 드물며, 현재는 아무 처리도 없으므로
어떤 값이든 개선이다.

**한계를 명시한다.** `twai_transmit()` 성공은 "TX 큐에 넣었다"는 뜻이지
"컨트롤러가 받았다"는 뜻이 아니다. 실패는 큐 포화, bus-off, 드라이버 미동작
같은 **로컬** 상태만 잡는다. 실제 도달 여부는 기존 피드백 freshness 검사가
판정한다. 즉 이 검사는 250 ms 피드백 타임아웃을 대체하지 않고, 그보다 빠른
보조 신호를 하나 더 얻는 것이다.

`state.can_commanded_current_L/R`은 실제 송신 성공 여부와 함께 기록하고,
`state.can_tx_fail_count_L/R`를 `include/state.h`에 추가해 진단에 노출한다.

### 5.6 스냅샷 신선도 검사

`published_ms`를 두고 `now - published_ms <= MOTOR_COMMAND_SNAPSHOT_MAX_AGE_MS`
일 때만 유효로 본다. 정상 동작에서 스냅샷 나이는 10~20 ms다.
`100`을 제안한다. `DEADMAN_MS = 200`보다 엄격한, 명령 경로 전용 검사가 된다.

`seq`는 프레임에 싣지 않는다. 진단 로그와 단위 테스트에서 같은 tick의 좌우
명령임을 확인하는 데 쓴다.

---

## 6. 수정 범위

LOCKED 파일 변경이 불가피하다. `AGENTS.md`는 `src/core/`, `src/logic/`,
`include/`, `src/main.cpp`, `platformio.ini` 수정을 금지하고 요청 시 사용자에게
먼저 확인하도록 규정한다. **아래 변경은 사용자 승인 후에만 적용한다.**

| 파일 | 상태 | 변경 |
|---|---|---|
| `src/modules/motor_command.h` | 신규 | 스냅샷/게이트/출력 타입 |
| `src/modules/motor_command.cpp` | 신규 | `motor_command_resolve()` 순수 로직 |
| `test/test_motor_command/test_motor_command.cpp` | 신규 | 단위 테스트 |
| `src/core/can_bus.h` | LOCKED | `publish_motor_command()` 선언 |
| `src/core/can_bus.cpp` | LOCKED | mux + 스냅샷, `life_task` 재작성, `send_torque` 시그니처, TX 실패 처리 |
| `src/core/app_wiring.cpp` | LOCKED | 태스크 순서 1줄, 게시 호출 추가 |
| `include/state.h` | LOCKED | `can_tx_fail_count_L/R` 진단 필드 |
| `src/modules/realcar_calibration.h` | 계약 헤더 | `MOTOR_COMMAND_SNAPSHOT_MAX_AGE_MS`, `MOTOR_TX_FAIL_LIMIT` |

제어 로직의 **수치 거동은 바뀌지 않는다.** 같은 입력에 같은 전류가 나간다.
바뀌는 것은 그 값이 어느 시점의 것인지에 대한 보장뿐이다.

---

## 7. 테스트 계획

### 7.1 신규 단위 테스트 (native, 하드웨어 불필요)

`test/test_motor_command/test_motor_command.cpp`

| # | 시나리오 | 기대 |
|---|---|---|
| 1 | Drive + 전부 허용 | 양측 `+4000 rpm`, `run = true` |
| 2 | Reverse + 전부 허용 | 양측 `-4000 rpm` |
| 3 | Drive + 양측 음전류(회생) | 양측 `0 rpm` |
| 4 | `safety_allow = false` | 양측 `0 A`, `run = false`, `0 rpm` |
| 5 | `gear = Neutral` / `Park` | `normal_allow = false` |
| 6 | `snapshot_fresh = false` | 양측 `0 A` |
| 7 | `reconnect_ramp_scale = 0.5` | 양측 동일 비율로 축소 |
| 8 | 좌우 전류 크기가 다르고 기어는 하나 | **양측 목표 rpm 부호 동일** (3.2 회귀 방지) |
| 9 | `left_a = NaN` | **양측** `0 A`, `run = false` (한쪽만 0 아님) |
| 10 | `propulsion_direction_armed = false` | 양측 `0 A` |
| 11 | `throttle_signal_valid = false` | 양측 `0 A` |

실행: `pio test -e native -f test_motor_command`

`platformio.ini`의 `build_dir`이 `C:\Users\jy02s\pio_build_vcu`로 고정되어 있다.
다른 사람의 경로이므로 로컬에서 실패할 수 있다. 실패하면 `PLATFORMIO_BUILD_DIR`
환경변수로 우회한다. `platformio.ini`는 LOCKED이므로 고치지 않는다.

### 7.2 회귀

- `pio test -e native` 전체 (기존 165 unit이 그대로 통과해야 한다)
- `pio run -e esp32dev` 빌드 통과

### 7.3 실차 확인 항목

- 부하 시험에서 좌우 명령의 `seq`가 항상 같은 값인지 로그로 확인
- 컷오프(페달 해제, 브레이크, HALT) 직후 좌우 명령이 동시에 0으로 떨어지는지
- 기어 전환 순간 좌우 목표 rpm 부호가 항상 같은지
- 한쪽 컨트롤러 전원 차단 시 양쪽 0 A, 복구 시 0→1 램프
- 연속 TX 실패 주입 시 `can_tx_fail_count_L/R` 증가와 링크 무효화 동작

---

## 8. 이 수정으로 해결되지 않는 것

- **좌우 CAN 프레임의 동시 도착.** CAN은 프레임을 직렬로 전송하므로 원리적으로
  불가능하다. 완화책은 두 프레임을 사이에 아무 작업 없이 연속으로 큐에 넣고,
  같은 life 바이트를 공유시켜 수신측과 로그에서 짝을 확인할 수 있게 하는 것뿐이다.
  life 바이트 공유는 이미 맞게 되어 있다 (`g_life++`가 두 송신 뒤, `can_bus.cpp:257`).
  **깨뜨리지 말 것.**
- **컴포넌트 테스트 경로의 공유 상태 경쟁.** 아래 별건 참조.
- **`state.can_commanded_*`의 역방향 경쟁.** core 1이 쓰고 core 0의 텔레메트리가
  읽는다. 표시 전용이라 무해하지만 문서화해 둔다.

---

## 9. 별건으로 분리된 발견

M2 검증 중 나온, 같은 계열이지만 범위가 다른 결함이다. 이 브랜치에서
고치지 않는다.

**컴포넌트 테스트 상태가 두 코어에서 모두 쓰인다.**

| 필드 | core 0 | core 1 |
|---|---|---|
| `component_test_active` | `debug_monitor.cpp:86` 쓰기 | `can_bus.cpp:182,213` 쓰기 |
| `component_test_normal_inhibit` | `debug_monitor.cpp:84` 쓰기 | `can_bus.cpp:150` 쓰기 |
| `component_test_release_ticks` | `debug_monitor.cpp:85` 쓰기 | `can_bus.cpp:146,153` 읽기-수정-쓰기 |

`release_ticks`는 core 1의 읽기-수정-쓰기 중간에 core 0이 0으로 덮어쓸 수 있다.
벤치 전용 기능이고 정상 주행에서는 비활성이지만, 올바른 구조는 core 1 소유의
상태 기계로 두고 core 0에서는 일회성 요청만 넘기는 것이다.

노션의 "Component Test·Time Sync → 시험 전용 빌드" 방향과 함께 처리하는 것이 맞다.

---

## 10. 노션 "반드시 유지" 항목과의 관계

M2 수정이 아래 항목을 **강화**한다. 제거하거나 우회하지 않는다.

| 노션 항목 | M2에서의 처리 |
|---|---|
| 양쪽 handshake 완료 전 토크 금지 | 그대로 유지. 송신 가드 `g_handshaked_L/R` 유지 |
| CAN stale → 양쪽 0 A | 스냅샷의 `controller_feedback_fresh`로 일관되게 평가 |
| 한쪽 fault → 양쪽 0 A | TX 연속 실패도 같은 경로로 연결 (5.5) |
| bus-off → handshake 무효 · 재연결 0→램프 | 그대로 유지. TX 실패가 이 경로를 재사용 |
| NaN/Inf 검사 + 모터별 상전류 clamp | **신규 추가.** 현재 최종 경로에 없다 (3.5) |
| 센서 오류 → TV만 OFF, 50:50 복귀 | 영향 없음. TV 상위 단계 |
| 10 kW 규정 전력 제한 | 영향 없음. `drive_supervisor` 유지 |
| 브레이크 입력 시 구동토크 차단 | 스냅샷의 `brake_active` 포함 |
| D/R 전환 정지 + 스로틀 해제 조건 | 스냅샷의 `propulsion_direction_armed` 포함 |

---

## 11. 다음 작업자를 위한 체크리스트

1. `git checkout fix/m2-command-snapshot` (base `origin/dev` @ `8498e2f`)
2. 줄 번호를 현재 파일에서 재확인한다. 이 문서는 `8498e2f` 기준이다.
3. `src/modules/motor_command.{h,cpp}`와 테스트를 **먼저** 작성하고
   `pio test -e native -f test_motor_command`를 통과시킨다. LOCKED 파일은
   이 단계에서 건드리지 않는다.
4. LOCKED 파일 변경은 사용자 승인 후 6절 표의 범위 안에서만 한다.
5. `pio test -e native` 전체와 `pio run -e esp32dev`를 통과시킨다.
6. 실차 확인은 7.3을 따른다. 스탠드 또는 통제된 환경에서 먼저 한다.
