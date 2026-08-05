// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
// [LOCKED] Domain types that enforce their own min/max at the type level.
// Integer template bounds only (C++17 limitation). Do not edit casually.

template <int LO, int HI>
class Clamped {
    float v_;
    void set(float x) { v_ = x < LO ? (float)LO : (x > HI ? (float)HI : x); }
public:
    Clamped(float x = 0.0f) { set(x); }
    Clamped &operator=(float x) { set(x); return *this; }
    operator float() const { return v_; }
};

using Percent   = Clamped<-100, 100>;  // throttle (진짜 백분율)
using Unit      = Clamped<-1, 1>;      // steering angle
using Pct0to100 = Clamped<0, 100>;     // brake
using Rpm       = Clamped<0, 6000>;    // wheel/motor speed

// 모터 상전류 [A]. 토크 명령은 백분율이 아니라 전류다 — EZkontrol CAN 프로토콜의
// Target Phase Current(0.1 A/bit) 가 그렇게 정의되어 있다.
// 범위는 HPM05KW 의 Max Phase Current 300 A. CAN 인코딩은 ±3200 A 까지 되지만
// 하드웨어가 못 내는 값을 타입이 허용할 이유가 없다.
using Amp       = Clamped<-300, 300>;  // motor phase current command
