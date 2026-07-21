// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/wheel_speed.h"

// ══════ [변경 표시] 원본 대비 바뀐 부분 ═══════════════════════════════════
//  • 계산식을 2줄 → 1식으로 정리 (결과값은 완전히 동일)
//    (원본: revs = pulse/ppr;  rpm = revs/(dt/1000)*60)
//  • 주석/보정 안내 추가 (기능 변화 없음)
// ═════════════════════════════════════════════════════════════════════════

// pulses_per_rev (WssCalib) = 바퀴 1회전당 센서 펄스 수 (하드웨어값).
//   자석/톱니 개수로 결정 — app_wiring 의 WSS_CAL 에서 주입 (현재 45).
//   ★실차 보정: 바퀴 1회전 돌려 나온 펄스 수를 세서 이 값을 맞출 것.
Rpm wheel_speed_compute(const WssReading &r, const WssCalib &c) {
    // 0으로 나누기/무효 입력 방지 (첫 읽기·정지 상태)
    if (r.dt_ms == 0 || c.pulses_per_rev <= 0.0f) return Rpm(0.0f);

    // rpm = (펄스수 / 펄스당회전) / (경과시간[초]) * 60
    //     = pulse_delta * 60000 / (pulses_per_rev * dt_ms)
    float rpm = (float)r.pulse_delta * 60000.0f          // ★변경(핵심): 한 식으로 정리(값 동일)
              / (c.pulses_per_rev * (float)r.dt_ms);

    // Rpm 타입이 0..6000 으로 클램프 (글리치성 과대값 2차 방어)
    return Rpm(rpm);
}