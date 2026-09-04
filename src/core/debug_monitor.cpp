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
#include "core/drivers/imu_driver.h"
#include "modules/realcar_calibration.h"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {
bool g_sync_arm_request = false;
bool g_sync_run_request = false;
bool g_sync_cancel_request = false;
char g_serial_line[64]{};
unsigned g_serial_line_length = 0U;

void reject_motor_test(const char *reason) {
    ++state.component_test_rejected_count;
    Serial.printf("[MOTOR_TEST] rejected: %s\n", reason);
}

void request_motor_test(bool left, bool right, float current_a,
                        unsigned duration_ms) {
    if (!std::isfinite(current_a) || current_a <= 0.0f ||
        current_a > realcar_cal::bringup::COMPONENT_TEST_CURRENT_MAX_PER_MOTOR_A) {
        reject_motor_test("current must be >0 and <=150 A per motor");
        return;
    }
    if (duration_ms < realcar_cal::bringup::COMPONENT_TEST_DURATION_MIN_MS ||
        duration_ms > realcar_cal::bringup::COMPONENT_TEST_DURATION_MAX_MS) {
        reject_motor_test("duration must be 100..3000 ms");
        return;
    }
    if (state.component_test_active || state.time_sync_armed ||
        state.time_sync_active) {
        reject_motor_test("another timed test is active or armed");
        return;
    }
    if ((float)state.throttle_pct >
            realcar_cal::bringup::THROTTLE_ARM_MAX_PCT ||
        state.brake_active || state.gear != Gear::Drive) {
        reject_motor_test("release throttle/brake and keep bring-up gear in D");
        return;
    }
    if (!state.throttle_signal_valid) {
        reject_motor_test("throttle signal raw is below the valid floor");
        return;
    }
    if (state.controller_fault_latched) {
        reject_motor_test("controller fault latch is set; power-cycle after diagnosis");
        return;
    }
    if (!component_test_safety_allowed()) {
        reject_motor_test("safety state is HALT; power-cycle after diagnosis");
        return;
    }
    if ((left && (!state.controller_handshaked_L ||
                  !state.controller_feedback_fresh_L ||
                  state.controller_fb2_L.any_fault() ||
                  state.controller_fb2_L.speed_mode ||
                  std::abs(state.controller_fb1_L.motor_speed_rpm) >
                      realcar_cal::bringup::COMPONENT_TEST_START_MAX_MOTOR_RPM)) ||
        (right && (!state.controller_handshaked_R ||
                   !state.controller_feedback_fresh_R ||
                   state.controller_fb2_R.any_fault() ||
                   state.controller_fb2_R.speed_mode ||
                   std::abs(state.controller_fb1_R.motor_speed_rpm) >
                       realcar_cal::bringup::COMPONENT_TEST_START_MAX_MOTOR_RPM))) {
        reject_motor_test("selected controller is not ready/fresh/fault-free/stopped");
        return;
    }

    state.component_test_left = left;
    state.component_test_right = right;
    state.component_test_current_a = current_a;
    state.component_test_deadline_ms = millis() + duration_ms;
    state.component_test_normal_inhibit = true;
    state.component_test_release_ticks = 0U;
    state.component_test_active = true;
    Serial.printf("[MOTOR_TEST] accepted: %s%s %.1f A for %u ms\n",
                  left ? "L" : "", right ? "R" : "", current_a,
                  duration_ms);
}

void accept_serial_command() {
    g_serial_line[g_serial_line_length] = '\0';
    float test_current_a = 0.0f;
    unsigned test_duration_ms = 0U;
    char trailing = '\0';
    if (std::strcmp(g_serial_line, "SYNC_ARM") == 0) {
        g_sync_arm_request = true;
        Serial.println("[SYNC] arm requested");
    } else if (std::strcmp(g_serial_line, "SYNC_RUN") == 0) {
        g_sync_run_request = true;
        Serial.println("[SYNC] run requested");
    } else if (std::strcmp(g_serial_line, "SYNC_CANCEL") == 0) {
        g_sync_cancel_request = true;
        Serial.println("[SYNC] cancel requested");
    } else if (std::sscanf(g_serial_line, "MOTOR_L %f %u %c",
                           &test_current_a, &test_duration_ms, &trailing) == 2) {
        request_motor_test(true, false, test_current_a, test_duration_ms);
    } else if (std::sscanf(g_serial_line, "MOTOR_R %f %u %c",
                           &test_current_a, &test_duration_ms, &trailing) == 2) {
        request_motor_test(false, true, test_current_a, test_duration_ms);
    } else if (std::sscanf(g_serial_line, "MOTOR_BOTH %f %u %c",
                           &test_current_a, &test_duration_ms, &trailing) == 2) {
        request_motor_test(true, true, test_current_a, test_duration_ms);
    } else if (g_serial_line_length != 0U) {
        Serial.println(
            "[CMD] use MOTOR_L|MOTOR_R|MOTOR_BOTH <A> <ms> or "
            "SYNC_ARM|SYNC_RUN|SYNC_CANCEL");
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
            Serial.println("[CMD] command too long");
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
    static uint32_t last_summary_ms = 0U;
    static bool was_fast_log_active = false;
    const uint32_t now = millis();
    const auto age_ms = [now](uint32_t timestamp) -> uint32_t {
        return timestamp == 0U ? 999999U : now - timestamp;
    };
    const uint32_t test_remaining_ms = state.component_test_active &&
        static_cast<int32_t>(state.component_test_deadline_ms - now) > 0
        ? state.component_test_deadline_ms - now : 0U;
    const bool fast_log_active = state.component_test_active ||
                                 state.time_sync_active;

    if (fast_log_active) {
        if (!was_fast_log_active) {
            Serial.println(
                "FAST_CSV,t_ms,mode,sel_l,sel_r,cmd_l,cmd_r,out_l,out_r,"
                "ibus_l,ibus_r,iph_l,iph_r,rpm_l,rpm_r,wss_rl,wss_rr,"
                "fb_l,fb_r,fault,rxq,rxmiss,buserr");
        }
        const bool motor_test = state.component_test_active;
        const float cmd_l = motor_test
            ? (state.component_test_left ? state.component_test_current_a : 0.0f)
            : state.time_sync_command_a;
        const float cmd_r = motor_test
            ? (state.component_test_right ? state.component_test_current_a : 0.0f)
            : state.time_sync_command_a;
        Serial.printf(
            "FAST_CSV,%u,%s,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%.0f,%.0f,%d,%d,%d,%u,%u,%u\n",
            now, motor_test ? "MOTOR" : "SYNC",
            motor_test ? state.component_test_left : 1,
            motor_test ? state.component_test_right : 1,
            cmd_l, cmd_r, state.can_commanded_current_L,
            state.can_commanded_current_R,
            state.controller_fb1_L.bus_current_a,
            state.controller_fb1_R.bus_current_a,
            state.controller_fb1_L.phase_current_a,
            state.controller_fb1_R.phase_current_a,
            state.controller_fb1_L.motor_speed_rpm,
            state.controller_fb1_R.motor_speed_rpm,
            (float)state.wheel_speed[WHEEL_RL],
            (float)state.wheel_speed[WHEEL_RR],
            state.controller_feedback_fresh_L,
            state.controller_feedback_fresh_R,
            state.controller_fault_latched,
            state.can_rx_queued_count, state.can_rx_missed_count,
            state.can_bus_error_count);
    } else {
        if (was_fast_log_active) {
            Serial.printf(
                "FAST_END,t_ms=%u,done=%u,abort=%u,reject=%u,rxMiss=%u,busErr=%u\n",
                now, state.component_test_completed_count,
                state.component_test_aborted_count,
                state.component_test_rejected_count,
                state.can_rx_missed_count, state.can_bus_error_count);
            last_summary_ms = 0U;
        }
        if (last_summary_ms == 0U || now - last_summary_ms >= 1000U) {
            last_summary_ms = now;
            const imu_driver::Diagnostics imu_diag = imu_driver::diagnostics();
            Serial.printf(
                "STAT arm=%d dm=%d hs=%d/%d fb=%d/%d fault=%d gear=%u/%u raw=%u thr=%d/%d/%.1f imu=%d sync=%d/%d test=%d/%u\n"
                "MCU V=%.1f/%.1f Ibus=%+.1f/%+.1f Iph=%+.1f/%+.1f rpm=%d/%d tempC=%d/%d,%d/%d err=%02X%02X%02X/%02X%02X%02X\n"
                "CAN state=%u age1=%u/%u age2=%u/%u q=%u peak=%u rxMiss=%u busErr=%u arbLost=%u txFail=%u | WSS=%.0f/%.0f/%.0f/%.0f pulse=%u/%u/%u/%u\n"
                "IMU valid=%d yaw=%+.2f ax=%+.3f ay=%+.3f rxBytes=%u frames=%u csErr=%u\n",
                torque_allowed(), can_bus::deadman_ok(),
                state.controller_handshaked_L, state.controller_handshaked_R,
                state.controller_feedback_fresh_L,
                state.controller_feedback_fresh_R,
                state.controller_fault_latched, (unsigned)state.gear,
                (unsigned)state.gear_sensed, (unsigned)state.gear_raw_adc,
                state.throttle_raw_adc, state.throttle_signal_valid,
                (float)state.throttle_pct,
                state.imu_valid, state.time_sync_armed, state.time_sync_active,
                state.component_test_active, test_remaining_ms,
                state.controller_fb1_L.bus_voltage_v,
                state.controller_fb1_R.bus_voltage_v,
                state.controller_fb1_L.bus_current_a,
                state.controller_fb1_R.bus_current_a,
                state.controller_fb1_L.phase_current_a,
                state.controller_fb1_R.phase_current_a,
                state.controller_fb1_L.motor_speed_rpm,
                state.controller_fb1_R.motor_speed_rpm,
                state.controller_fb2_L.controller_temp_c,
                state.controller_fb2_R.controller_temp_c,
                state.controller_fb2_L.motor_temp_c,
                state.controller_fb2_R.motor_temp_c,
                state.controller_fb2_L.error1, state.controller_fb2_L.error2,
                state.controller_fb2_L.error3, state.controller_fb2_R.error1,
                state.controller_fb2_R.error2, state.controller_fb2_R.error3,
                (unsigned)state.can_state,
                age_ms(state.controller_fb1_last_ms_L),
                age_ms(state.controller_fb1_last_ms_R),
                age_ms(state.controller_fb2_last_ms_L),
                age_ms(state.controller_fb2_last_ms_R),
                state.can_rx_queued_count, state.can_rx_queue_peak,
                state.can_rx_missed_count, state.can_bus_error_count,
                state.can_arb_lost_count, state.can_tx_failed_count,
                (float)state.wheel_speed[WHEEL_FL],
                (float)state.wheel_speed[WHEEL_FR],
                (float)state.wheel_speed[WHEEL_RL],
                (float)state.wheel_speed[WHEEL_RR],
                state.wheel_pulse_total[WHEEL_FL],
                state.wheel_pulse_total[WHEEL_FR],
                state.wheel_pulse_total[WHEEL_RL],
                state.wheel_pulse_total[WHEEL_RR],
                state.imu_valid, state.yaw_rate, state.accel_x, state.accel_y,
                imu_diag.rx_bytes, imu_diag.valid_mtdata2_frames,
                imu_diag.checksum_errors);
        }
    }
    was_fast_log_active = fast_log_active;
#endif
}
