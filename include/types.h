// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
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

template <int LO, int HI>
class Clamped {
    float v_;
    void set(float x) { v_ = x < LO ? (float)LO : (x > HI ? (float)HI : x); }
public:
    constexpr Clamped() : v_(0.0f) {}
    explicit Clamped(float x) { set(x); }
    explicit operator float() const { return v_; }
};

using Percent   = Clamped<-100, 100>;  // true percentage values
using Unit      = Clamped<-1, 1>;      // steering angle
using Pct0to100 = Clamped<0, 100>;     // brake
using Rpm       = Clamped<0, 6000>;    // wheel/motor speed

// EZkontrol Target Phase Current is an ampere command, not a percentage.
// The domain range follows the configured short-duration software command
// ceiling; continuous and paddock limits are enforced separately.
using Amp       = Clamped<-500, 500>;
