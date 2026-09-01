// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "core/debug_monitor.h"
#include <Arduino.h>
#include "can_bus.h"
#include "safety_logic.h"
#include "state.h"
#include <cstring>

namespace {
bool g_sync_arm_request = false;
bool g_sync_run_request = false;
bool g_sync_cancel_request = false;
char g_serial_line[32]{};
unsigned g_serial_line_length = 0U;

void accept_serial_command() {
    g_serial_line[g_serial_line_length] = '\0';
    if (std::strcmp(g_serial_line, "SYNC_ARM") == 0) {
        g_sync_arm_request = true;
        Serial.println("[SYNC] arm requested");
    } else if (std::strcmp(g_serial_line, "SYNC_RUN") == 0) {
        g_sync_run_request = true;
        Serial.println("[SYNC] run requested");
    } else if (std::strcmp(g_serial_line, "SYNC_CANCEL") == 0) {
        g_sync_cancel_request = true;
        Serial.println("[SYNC] cancel requested");
    } else if (g_serial_line_length != 0U) {
        Serial.println("[SYNC] unknown command; use SYNC_ARM, SYNC_RUN, or SYNC_CANCEL");
    }
    g_serial_line_length = 0U;
}

void poll_serial_commands() {
    while (Serial.available() > 0) {
        const char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            if (g_serial_line_length != 0U) accept_serial_command();
            continue;
        }
        if (g_serial_line_length + 1U < sizeof(g_serial_line)) {
            g_serial_line[g_serial_line_length++] = c;
        } else {
            g_serial_line_length = 0U;
            Serial.println("[SYNC] command too long");
        }
    }
}
}

bool debug_consume_time_sync_arm_request() {
    const bool value = g_sync_arm_request;
    g_sync_arm_request = false;
    return value;
}

bool debug_consume_time_sync_run_request() {
    const bool value = g_sync_run_request;
    g_sync_run_request = false;
    return value;
}

bool debug_consume_time_sync_cancel_request() {
    const bool value = g_sync_cancel_request;
    g_sync_cancel_request = false;
    return value;
}

void debug_update() {
    poll_serial_commands();
#if DEBUG_MONITOR
    Serial.printf(
        "ARM=%d HS=%d DM=%d FB=%d FLT=%d | gear=%u CMD=%d TV=%d PAD=%d RG=%d\n"
        "  thrRaw=%4d thr=%5.1f brk=%5.1f str=%+5.2f yaw=%+7.1f | req=%+6.1f/%+6.1f out=%+6.1f/%+6.1f\n"
        "  MCU: V=%5.1f/%5.1f Ibus=%+6.1f/%+6.1f Iph=%+6.1f/%+6.1f rpm=%+5d/%+5d Tctrl=%3d/%3d Tmot=%3d/%3d\n"
        "  LIMIT: Pbus=%7.0f Pest=%7.0f scale=%4.2f power=%d thermal=%d | SYNC arm=%d run=%d cmd=%4.1f done=%u abort=%u\n"
        "  WSS(FL/FR/RL/RR)=%5.0f/%5.0f/%5.0f/%5.0f rpm  V=%5.2f m/s %s\n"
        "  TV: yaw*=%+6.1f Mz=%+7.1f Fz(L/R)=%6.0f/%6.0f Tmax(L/R)=%7.0f/%7.0f\n",
        torque_allowed(), can_bus::handshaked(), can_bus::deadman_ok(),
        state.controller_feedback_fresh, state.controller_fault_latched,
        (unsigned)state.gear, state.cluster_cmd_alive, state.tv_enable_requested,
        state.paddock_active, state.regen_auto_requested,
        state.throttle_raw_adc, (float)state.throttle_pct, (float)state.brake_pct,
        (float)state.steering_angle,
        state.yaw_rate,
        (float)state.requested_torque_L, (float)state.requested_torque_R,
        (float)state.torque_L, (float)state.torque_R,
        state.controller_fb1_L.bus_voltage_v, state.controller_fb1_R.bus_voltage_v,
        state.controller_fb1_L.bus_current_a, state.controller_fb1_R.bus_current_a,
        state.controller_fb1_L.phase_current_a, state.controller_fb1_R.phase_current_a,
        state.controller_fb1_L.motor_speed_rpm, state.controller_fb1_R.motor_speed_rpm,
        state.controller_fb2_L.controller_temp_c, state.controller_fb2_R.controller_temp_c,
        state.controller_fb2_L.motor_temp_c, state.controller_fb2_R.motor_temp_c,
        state.measured_bus_power_w, state.estimated_input_power_w,
        state.drive_limit_scale, state.power_limited, state.thermal_limited,
        state.time_sync_armed, state.time_sync_active,
        state.time_sync_command_a, state.time_sync_completed_count,
        state.time_sync_aborted_count,
        (float)state.wheel_speed[WHEEL_FL], (float)state.wheel_speed[WHEEL_FR],
        (float)state.wheel_speed[WHEEL_RL], (float)state.wheel_speed[WHEEL_RR],
        state.vehicle_speed_mps, state.vehicle_speed_valid ? "ok" : "INVALID",
        state.desired_yaw_rate, state.yaw_moment, state.fz_L, state.fz_R,
        state.max_torque_L, state.max_torque_R);
#endif
}
