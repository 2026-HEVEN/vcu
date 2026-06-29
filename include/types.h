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

using Percent   = Clamped<-100, 100>;  // throttle, torque %
using Unit      = Clamped<-1, 1>;      // steering angle
using Pct0to100 = Clamped<0, 100>;     // brake
using Rpm       = Clamped<0, 6000>;    // wheel/motor speed
