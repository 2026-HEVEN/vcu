// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "core/debug_monitor.h"
#include <Arduino.h>
#include "state.h"

void debug_update() {
#if DEBUG_MONITOR
    Serial.printf(
        "thr=%5.1f brk=%5.1f str=%+5.2f yaw=%+7.1f wss=%6.1f | totT=%+6.1f L=%+6.1f R=%+6.1f\n"
        "  TV: yaw*=%+6.1f Mz=%+7.1f Fz(L/R)=%6.0f/%6.0f Tmax(L/R)=%7.0f/%7.0f\n",
        (float)state.throttle_pct, (float)state.brake_pct, (float)state.steering_angle,
        state.yaw_rate, (float)state.wheel_speed,
        state.total_torque, (float)state.torque_L, (float)state.torque_R,
        state.desired_yaw_rate, state.yaw_moment, state.fz_L, state.fz_R,
        state.max_torque_L, state.max_torque_R);
#endif
}
