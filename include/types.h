// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include <cstdint>
// [LOCKED] Domain types that enforce their own min/max at the type level.
// Integer template bounds only (C++17 limitation). Do not edit casually.
//
// 차원 계약: float <-> 도메인 타입 변환은 양방향 모두 explicit이다.
// 중간 계산은 생 float로 자유롭게 하되, 경계를 넘을 때 Amp(x) / (float)a 를
// 쓰도록 강제해 "이 차원의 값을 의도했다"를 코드에 남긴다. 암묵 변환을 열어두면
// `float mz = ...; Amp a = mz;` 처럼 N.m을 암페어 자리에 넣어도 통과한다
// (#15: 토크 명령을 Percent로 선언한 사고가 이 계열이다).
// 기본 생성자는 explicit이 아니다 -- `TVAllocOutput a{};` 같은 값초기화를
// 지켜야 하고, 값이 0이라 차원을 오해할 여지가 없다.
// float 대입 연산자도 두지 않는다. 생성만 막고 대입을 열어두면
// `Amp a; a = mz;` 로 그대로 새기 때문이다. 대입도 a = Amp(x) 를 쓴다.

// ── 포화 진단 ────────────────────────────────────────────────────────────
// 클램프는 지금까지 완전히 무음이었다. 주행 중 Amp가 500 A에 몇 번 걸렸는지,
// 실제 요구가 얼마였는지 알 방법이 없었다. 잘릴 때마다 기록만 남긴다 --
// 제어 루프(100 Hz) 안에서 I/O를 하면 스로틀 고착 시 Serial 폭주로 워치독이
// 물린다. 문자열 출력은 debug_monitor가 자기 주기에 한다.
//
// 호출 지점은 __builtin_FILE/LINE/FUNCTION 을 기본 인자로 받아 잡는다. 기본
// 인자는 호출측에서 평가되므로 std::source_location(C++20) 과 같은 효과를
// C++17에서 얻으면서 호출 코드는 전혀 바뀌지 않는다. GCC 8.4(Xtensa)와
// clang 양쪽에서 확인했다.
// ClampEvent에는 기본 멤버 초기화자를 넣지 말 것. ESP32 Xtensa GCC 8.4는
// 기본 멤버 초기화자가 있으면 이 struct를 aggregate로 취급하지 않아 아래
// record()의 `ClampEvent{raw, file, fn, line}` 리스트 초기화가 컴파일 실패한다
// (clang/native는 통과하므로 native 테스트만으로는 발견되지 않는다).
// TVInput이 같은 이유로 같은 제약을 지고 있다 -- modules/torque_vectoring.h 참조.
struct ClampEvent {
    float       raw;              // 잘리기 전 원래 값
    const char *file;
    const char *fn;
    int         line;
};
struct ClampStats {
    uint32_t   high_count = 0;    // 상한에 걸린 횟수
    uint32_t   low_count  = 0;    // 하한에 걸린 횟수
    ClampEvent high_worst{};      // 상한 초과분 중 raw가 가장 큰 사건
    ClampEvent low_worst{};       // 하한 미만분 중 raw가 가장 작은 사건
    ClampEvent last{};            // 가장 최근에 잘린 사건
};

template <int LO, int HI>
class Clamped {
    float v_;

    // 인스턴스화마다 하나씩 생긴다 -- Amp, Percent, Unit 이 각자 따로 센다.
    // 객체 크기는 4바이트 그대로이고 가상 함수도 없다(aggregate 유지).
    static inline ClampStats stats_{};

    static void record(ClampEvent &slot, float raw,
                       const char *file, const char *fn, int line) {
        slot = ClampEvent{raw, file, fn, line};
    }

    void set(float x, const char *file, const char *fn, int line) {
        if (x < LO) {
            v_ = (float)LO;
            ++stats_.low_count;
            if (stats_.low_count == 1 || x < stats_.low_worst.raw)
                record(stats_.low_worst, x, file, fn, line);
            record(stats_.last, x, file, fn, line);
        } else if (x > HI) {
            v_ = (float)HI;
            ++stats_.high_count;
            if (stats_.high_count == 1 || x > stats_.high_worst.raw)
                record(stats_.high_worst, x, file, fn, line);
            record(stats_.last, x, file, fn, line);
        } else {
            v_ = x;
        }
    }
public:
    constexpr Clamped() : v_(0.0f) {}
    explicit Clamped(float x,
                     const char *file = __builtin_FILE(),
                     const char *fn   = __builtin_FUNCTION(),
                     int line         = __builtin_LINE()) {
        set(x, file, fn, line);
    }
    explicit operator float() const { return v_; }

    static ClampStats clamp_stats() { return stats_; }
    static void reset_clamp_stats() { stats_ = ClampStats{}; }
};

using Percent   = Clamped<-100, 100>;  // true percentage values
using Unit      = Clamped<-1, 1>;      // steering angle
using Pct0to100 = Clamped<0, 100>;     // brake
using Rpm       = Clamped<0, 6000>;    // wheel/motor speed

// EZkontrol Target Phase Current is an ampere command, not a percentage.
// The domain range follows the configured short-duration software command
// ceiling; continuous and paddock limits are enforced separately.
using Amp       = Clamped<-500, 500>;
