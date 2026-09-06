// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "core/can_bus.h"
#include <Arduino.h>
#include <cstring>
#include <cmath>
#include "driver/twai.h"
#include "can_protocol.h"
#include "state.h"
#include "safety_logic.h"   // torque_allowed()
#include "core/board_pins.h"
#include "modules/realcar_calibration.h"

// [LOCKED] Bit layout of EZkontrol control frames follows
// EZkontrol-CANBUS-MCU-to-VCU.pdf and the reference 2026/Can_driver/CAN_DRIVER.ino.

// NOTE (deadman scope): note_command() is currently called every scheduler
// pass from app_wiring (torque_vectoring_update), so deadman_ok() effectively
// means "loop()/scheduler is still alive within 200ms", NOT "a fresh, valid
// command source exists". This is acceptable for the skeleton (it catches a
// hung scheduler). When CAN RX/handshake + throttle plausibility are
// implemented, gate note_command() on a genuinely valid command instead.
//
// NOTE (concurrency): state.torque_L/R (float) are written by loop()/scheduler
// and read here on the core-1 life task. Relies on 32-bit aligned float access
// being word-atomic on ESP32; a one-cycle-stale value is benign and torque is
// force-zeroed when not allowed. g_life is only touched inside life_task.

namespace {
    constexpr uint32_t DEADMAN_MS = 200;
    constexpr int DRIVE_TARGET_SPEED_RPM = 4000;
    constexpr int REGEN_TARGET_SPEED_RPM = 0;
    volatile uint32_t  g_last_cmd_ms = 0;
    volatile bool      g_handshaked  = false;
    volatile bool      g_handshaked_L = false;
    volatile bool      g_handshaked_R = false;
    volatile bool      g_reconnect_inhibit = false;
    volatile bool      g_reconnect_ramp_active = false;
    volatile uint32_t  g_reconnect_ramp_start_ms = 0U;
    uint32_t            g_handshake_reply_ms_L = 0U;
    uint32_t            g_handshake_reply_ms_R = 0U;
    bool                g_bus_recovery_pending = false;
    uint8_t            g_life = 0;
    uint8_t            g_status_life = 0;

    void invalidate_controller_link(bool left, const char *reason) {
        bool was_handshaked = false;
        if (left) {
            was_handshaked = g_handshaked_L;
            g_handshaked_L = false;
            state.controller_handshaked_L = false;
            state.controller_feedback_fresh_L = false;
            state.controller_fb1_last_ms_L = 0U;
            state.controller_fb2_last_ms_L = 0U;
        } else {
            was_handshaked = g_handshaked_R;
            g_handshaked_R = false;
            state.controller_handshaked_R = false;
            state.controller_feedback_fresh_R = false;
            state.controller_fb1_last_ms_R = 0U;
            state.controller_fb2_last_ms_R = 0U;
        }
        state.controller_feedback_fresh = false;
        g_reconnect_inhibit = true;
        g_reconnect_ramp_active = false;

        if (was_handshaked) {
            Serial.printf("[CAN] controller %c link lost (%s); awaiting handshake\n",
                          left ? 'L' : 'R', reason);
        }
    }

    void invalidate_all_controller_links(const char *reason) {
        invalidate_controller_link(true, reason);
        invalidate_controller_link(false, reason);
        g_handshaked = false;
    }

    void transmit_ext(uint32_t id, const uint8_t data[8]) {
        twai_message_t m = {};
        m.identifier = id;
        m.extd = 1;
        m.data_length_code = 8;
        for (int i = 0; i < 8; ++i) m.data[i] = data[i];
        twai_transmit(&m, pdMS_TO_TICKS(5));
    }

    void send_torque(uint32_t id, float amps, bool running) {
        // Confirmed on the vehicle: propulsion uses +4000 rpm while regen
        // uses 0 rpm. Byte4 must be 0x01 or EZkontrol remains HALTED.
        const int target_rpm = !running ? 0
            : (state.gear == Gear::Reverse ? -DRIVE_TARGET_SPEED_RPM
                : (amps < 0.0f ? REGEN_TARGET_SPEED_RPM
                               : DRIVE_TARGET_SPEED_RPM));
        uint8_t data[8];
        encode_motor_control(amps, target_rpm, running, g_life, data);

        twai_message_t m = {};
        m.identifier = id; m.extd = 1; m.data_length_code = 8;
        for (int i = 0; i < 8; ++i) m.data[i] = data[i];
        twai_transmit(&m, pdMS_TO_TICKS(5));
    }

    void life_task(void *) {
        const TickType_t period = pdMS_TO_TICKS(
            realcar_cal::bringup::MOTOR_COMMAND_PERIOD_MS);
        TickType_t next = xTaskGetTickCount();
        for (;;) {
            const uint32_t now = millis();
            const bool scheduler_alive = (now - g_last_cmd_ms < DEADMAN_MS);
            if (g_reconnect_inhibit) {
                const bool protocol_ready =
                    realcar_cal::bringup::REQUIRE_BOTH_MOTOR_CONTROLLERS
                        ? (g_handshaked_L && g_handshaked_R)
                        : (g_handshaked_L || g_handshaked_R);
                const bool feedback_ready =
                    realcar_cal::bringup::REQUIRE_BOTH_MOTOR_CONTROLLERS
                        ? (state.controller_feedback_fresh_L &&
                           state.controller_feedback_fresh_R)
                        : (state.controller_feedback_fresh_L ||
                           state.controller_feedback_fresh_R);
                if (protocol_ready && feedback_ready) {
                    g_reconnect_inhibit = false;
                    g_reconnect_ramp_active = true;
                    g_reconnect_ramp_start_ms = now;
                    Serial.println(
                        "[CAN] controller link restored; torque ramp started");
                }
            }
            if (state.component_test_normal_inhibit) {
                if (!state.component_test_active &&
                    state.throttle_signal_valid &&
                    (float)state.throttle_pct <=
                        realcar_cal::bringup::THROTTLE_ARM_MAX_PCT) {
                    if (state.component_test_release_ticks <
                        realcar_cal::bringup::COMPONENT_TEST_RELEASE_TICKS) {
                        ++state.component_test_release_ticks;
                    }
                    if (state.component_test_release_ticks >=
                        realcar_cal::bringup::COMPONENT_TEST_RELEASE_TICKS) {
                        state.component_test_normal_inhibit = false;
                    }
                } else {
                    state.component_test_release_ticks = 0U;
                }
            }
            const bool propulsion_gear =
                state.gear == Gear::Drive || state.gear == Gear::Reverse;
            bool normal_allow = torque_allowed() && scheduler_alive &&
                                state.throttle_signal_valid &&
                                !g_reconnect_inhibit &&
                                !state.component_test_normal_inhibit &&
                                propulsion_gear &&
                                state.propulsion_direction_armed;
            float l = normal_allow ? (float)state.torque_L : 0.0f;
            float r = normal_allow ? (float)state.torque_R : 0.0f;
            bool run_l = normal_allow;
            bool run_r = normal_allow;

            // Branch-only component test path. While a test is active it has
            // exclusive ownership of both command outputs: the unselected
            // motor is explicitly HALTED and normal throttle cannot mix in.
            if (state.component_test_active) {
                normal_allow = false;
                l = 0.0f;
                r = 0.0f;
                run_l = false;
                run_r = false;

                const bool before_deadline =
                    static_cast<int32_t>(state.component_test_deadline_ms - now) > 0;
                if (!before_deadline) {
                    state.component_test_active = false;
                    ++state.component_test_completed_count;
                } else {
                    const bool common_ok = scheduler_alive &&
                        component_test_safety_allowed() &&
                        state.throttle_signal_valid &&
                        !state.controller_fault_latched &&
                        (float)state.throttle_pct <=
                            realcar_cal::bringup::THROTTLE_ARM_MAX_PCT &&
                        !state.brake_active && state.gear == Gear::Drive;
                    const bool left_ok = !state.component_test_left ||
                        (g_handshaked_L && state.controller_feedback_fresh_L &&
                         !state.controller_fb2_L.any_fault() &&
                         state.controller_fb2_L.controller_temp_c <
                            realcar_cal::bringup::CONTROLLER_CUTOFF_C &&
                         state.controller_fb2_L.motor_temp_c <
                            realcar_cal::bringup::MOTOR_CUTOFF_C &&
                         !state.controller_fb2_L.speed_mode &&
                         std::fabs(state.controller_fb1_L.phase_current_a) <
                            realcar_cal::bringup::PHASE_CURRENT_HARD_CUTOFF_A);
                    const bool right_ok = !state.component_test_right ||
                        (g_handshaked_R && state.controller_feedback_fresh_R &&
                         !state.controller_fb2_R.any_fault() &&
                         state.controller_fb2_R.controller_temp_c <
                            realcar_cal::bringup::CONTROLLER_CUTOFF_C &&
                         state.controller_fb2_R.motor_temp_c <
                            realcar_cal::bringup::MOTOR_CUTOFF_C &&
                         !state.controller_fb2_R.speed_mode &&
                         std::fabs(state.controller_fb1_R.phase_current_a) <
                            realcar_cal::bringup::PHASE_CURRENT_HARD_CUTOFF_A);
                    if (!common_ok || !left_ok || !right_ok) {
                        state.component_test_active = false;
                        ++state.component_test_aborted_count;
                    } else {
                        const float test_a = std::fmax(0.0f, std::fmin(
                            state.component_test_current_a,
                            realcar_cal::bringup::COMPONENT_TEST_CURRENT_MAX_PER_MOTOR_A));
                        if (state.component_test_left) {
                            l = test_a;
                            run_l = true;
                        }
                        if (state.component_test_right) {
                            r = test_a;
                            run_r = true;
                        }
                    }
                }
            }

            // Reconnection is automatic even with a held pedal, but the
            // recovered demand is applied to both motors through the same
            // zero-to-one ramp to avoid a sudden torque step or imbalance.
            if (normal_allow && g_reconnect_ramp_active) {
                const uint32_t elapsed_ms = now - g_reconnect_ramp_start_ms;
                const uint32_t ramp_ms =
                    realcar_cal::bringup::MOTOR_RECONNECT_RAMP_MS;
                if (elapsed_ms >= ramp_ms) {
                    g_reconnect_ramp_active = false;
                    Serial.println("[CAN] controller reconnect torque ramp complete");
                } else {
                    const float ramp_scale =
                        (float)elapsed_ms / (float)ramp_ms;
                    l *= ramp_scale;
                    r *= ramp_scale;
                }
            }

            // Do not place normal command frames on a controller ID before
            // that controller has completed its 0x55/0xAA handshake.
            state.can_commanded_current_L = l;
            state.can_commanded_current_R = r;
            state.can_commanded_running_L = run_l;
            state.can_commanded_running_R = run_r;
            if (g_handshaked_L) send_torque(CAN_ID_TORQUE_L, l, run_l);
            if (g_handshaked_R) send_torque(CAN_ID_TORQUE_R, r, run_r);
            g_life++;
            vTaskDelayUntil(&next, period);   // configured exact cadence
        }
    }
}

namespace can_bus {

void begin() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        static_cast<gpio_num_t>(board_pins::CAN_TX),
        static_cast<gpio_num_t>(board_pins::CAN_RX),
        TWAI_MODE_NORMAL);
    g.rx_queue_len = realcar_cal::bringup::CAN_RX_QUEUE_LENGTH;
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g, &t, &f);
    twai_start();
}

void start_life_task() {
    // High priority, pinned to core 1, away from the loop()/scheduler.
    xTaskCreatePinnedToCore(life_task, "can_life", 4096, nullptr, 20, nullptr, 1);
}


void send_vehicle_speed() {
    uint8_t data[8];
    const float speed_kph = state.vehicle_speed_mps * 3.6f;
    encode_vcu_vehicle_speed(speed_kph, state.vehicle_speed_valid, data);
    transmit_ext(CAN_ID_VCU_VEHICLE_SPEED, data);
}

void send_cluster_status() {
    uint8_t data[8];
    const bool hv_active = state.controller_feedback_fresh &&
        (state.controller_fb1_L.bus_voltage_v > 20.0f ||
         state.controller_fb1_R.bus_voltage_v > 20.0f);
    // SOC remains invalid because the current BLE-forwarded BMS frame is
    // diagnostic/display-only and its source parser is not authoritative.
    const uint8_t throttle_pct = static_cast<uint8_t>(
        (float)state.throttle_pct + 0.5f);
    encode_vcu_cluster_status(static_cast<uint8_t>(state.gear),
                              state.brake_active, hv_active,
                              state.paddock_active,
                              false, 0,
                              state.throttle_signal_valid, throttle_pct,
                              g_status_life++, data);
    transmit_ext(CAN_ID_VCU_CLUSTER_STATUS, data);
}

void send_sensor_telemetry() {
    uint8_t data[8];
    encode_vcu_steering((float)state.steering_angle, data);
    transmit_ext(CAN_ID_VCU_STEERING, data);
    encode_vcu_imu(state.yaw_rate, state.accel_x, state.accel_y, data);
    transmit_ext(CAN_ID_VCU_IMU, data);
}

void poll_rx() {
    // EZkontrol handshake (docs/CAN_PROTOCOL.md §6): each controller sends its
    // feedback-Part-I ID (CAN_ID_FB1_L/R) with all 8 data bytes = 0x55 at
    // startup (50ms/20Hz) until the VCU replies on the matching torque-command
    // ID (CAN_ID_TORQUE_L/R) with all 8 data bytes = 0xAA. That reply frame
    // carries no real torque; the life_task's configured-period torque frames
    // take over once running. A 0x55-pattern frame is a handshake probe, not real
    // feedback, so it must be intercepted before feedback parsing.
    static const uint8_t HANDSHAKE_PATTERN[8] = {0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};
    twai_status_info_t before_drain{};
    if (twai_get_status_info(&before_drain) == ESP_OK &&
        before_drain.msgs_to_rx > state.can_rx_queue_peak) {
        state.can_rx_queue_peak = before_drain.msgs_to_rx;
    }
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) {
        if (!m.extd) continue;
        const bool from_l = (m.identifier == CAN_ID_FB1_L);
        const bool from_r = (m.identifier == CAN_ID_FB1_R);

        if ((from_l || from_r) && m.data_length_code == 8 &&
            memcmp(m.data, HANDSHAKE_PATTERN, 8) == 0) {
            // A new probe while already handshaked means the controller reset
            // or timed out. Inhibit propulsion before acknowledging it so a
            // held pedal cannot resume torque immediately after recovery.
            if ((from_l && g_handshaked_L) || (from_r && g_handshaked_R)) {
                invalidate_controller_link(from_l, "new handshake probe");
            }
            twai_message_t reply = {};
            reply.identifier = from_l ? CAN_ID_TORQUE_L : CAN_ID_TORQUE_R;
            reply.extd = 1;
            reply.data_length_code = 8;
            memset(reply.data, 0xAA, 8);
            const esp_err_t tx_result = twai_transmit(&reply, pdMS_TO_TICKS(5));
            if (tx_result == ESP_OK) {
                if (from_l) {
                    g_handshaked_L = true;
                    state.controller_handshaked_L = true;
                    g_handshake_reply_ms_L = millis();
                } else {
                    g_handshaked_R = true;
                    state.controller_handshaked_R = true;
                    g_handshake_reply_ms_R = millis();
                }
                Serial.printf("[CAN] controller %c handshake reply sent\n",
                              from_l ? 'L' : 'R');
            } else {
                Serial.printf("[CAN] controller %c handshake reply failed: %d\n",
                              from_l ? 'L' : 'R', static_cast<int>(tx_result));
            }
            continue;
        }

        if (m.identifier == CAN_ID_CLUSTER_CMD && m.data_length_code == 8) {
            ClusterCommandRequest cmd = decode_cluster_command(m.data);
            // Cluster UI/PCB calls this switch "TC", but the agreed project
            // meaning is the torque-vectoring enable request.
            state.tv_enable_requested = cmd.tv_enabled;
            state.regen_auto_requested = cmd.regen_auto_enabled;
            state.paddock_requested = cmd.paddock_request;
            state.debug_requested = cmd.debug_enabled;
            state.cluster_cmd_last_rx_ms = millis();
            state.cluster_cmd_alive = true;
            continue;
        }

        const uint32_t now = millis();
        if (m.data_length_code == 8 && m.identifier == CAN_ID_FB1_L) {
            state.controller_fb1_L = decode_controller_feedback_part1(m.data);
            state.controller_fb1_last_ms_L = now;
            if (std::fabs(state.controller_fb1_L.phase_current_a) >
                realcar_cal::bringup::PHASE_CURRENT_HARD_CUTOFF_A) {
                state.controller_fault_latched = true;
            }
            continue;
        }
        if (m.data_length_code == 8 && m.identifier == CAN_ID_FB1_R) {
            state.controller_fb1_R = decode_controller_feedback_part1(m.data);
            state.controller_fb1_last_ms_R = now;
            if (std::fabs(state.controller_fb1_R.phase_current_a) >
                realcar_cal::bringup::PHASE_CURRENT_HARD_CUTOFF_A) {
                state.controller_fault_latched = true;
            }
            continue;
        }
        if (m.data_length_code == 8 && m.identifier == CAN_ID_FB2_L) {
            state.controller_fb2_L = decode_controller_feedback_part2(m.data);
            state.controller_fb2_last_ms_L = now;
            if (state.controller_fb2_L.any_fault()) state.controller_fault_latched = true;
            continue;
        }
        if (m.data_length_code == 8 && m.identifier == CAN_ID_FB2_R) {
            state.controller_fb2_R = decode_controller_feedback_part2(m.data);
            state.controller_fb2_last_ms_R = now;
            if (state.controller_fb2_R.any_fault()) state.controller_fault_latched = true;
            continue;
        }
        if (m.data_length_code == 8 && m.identifier == CAN_ID_CLUSTER_BMS_STATUS) {
            const ClusterBmsStatus bms = decode_cluster_bms_status(m.data);
            state.pack_data_valid = bms.valid && bms.ble_connected;
            state.pack_soc = (float)bms.soc_pct / 100.0f;
            state.pack_voltage_v = bms.pack_voltage_v;
            state.pack_current_a = bms.pack_current_a;
            state.pack_temperature_c = bms.temperature_c;
            state.bms_last_rx_ms = now;
            continue;
        }
    }
    const uint32_t now = millis();
    const auto fresh = [now](uint32_t timestamp, uint32_t max_age_ms) {
        return timestamp != 0 && (now - timestamp) <= max_age_ms;
    };
    const uint32_t feedback_stale_ms =
        (uint32_t)realcar_cal::bringup::CONTROLLER_FEEDBACK_STALE_MS;
    state.controller_feedback_fresh_L =
        fresh(state.controller_fb1_last_ms_L, feedback_stale_ms) &&
        fresh(state.controller_fb2_last_ms_L, feedback_stale_ms);
    state.controller_feedback_fresh_R =
        fresh(state.controller_fb1_last_ms_R, feedback_stale_ms) &&
        fresh(state.controller_fb2_last_ms_R, feedback_stale_ms);
    state.controller_feedback_fresh =
        state.controller_feedback_fresh_L &&
        state.controller_feedback_fresh_R;

    // The 250 ms freshness check above removes torque immediately. If either
    // required feedback part is still absent at the longer timeout, stop that
    // side's normal command frames. The controller can then enter its
    // documented timeout path and issue a fresh 0x55 handshake probe.
    const uint32_t rehandshake_timeout_ms =
        realcar_cal::bringup::CONTROLLER_REHANDSHAKE_TIMEOUT_MS;
    if (g_handshaked_L &&
        (now - g_handshake_reply_ms_L) > rehandshake_timeout_ms &&
        (!fresh(state.controller_fb1_last_ms_L, rehandshake_timeout_ms) ||
         !fresh(state.controller_fb2_last_ms_L, rehandshake_timeout_ms))) {
        invalidate_controller_link(true, "feedback timeout");
    }
    if (g_handshaked_R &&
        (now - g_handshake_reply_ms_R) > rehandshake_timeout_ms &&
        (!fresh(state.controller_fb1_last_ms_R, rehandshake_timeout_ms) ||
         !fresh(state.controller_fb2_last_ms_R, rehandshake_timeout_ms))) {
        invalidate_controller_link(false, "feedback timeout");
    }

    twai_status_info_t can_status{};
    if (twai_get_status_info(&can_status) == ESP_OK) {
        state.can_tx_failed_count = can_status.tx_failed_count;
        state.can_rx_missed_count = can_status.rx_missed_count;
        state.can_bus_error_count = can_status.bus_error_count;
        state.can_arb_lost_count = can_status.arb_lost_count;
        state.can_rx_queued_count = can_status.msgs_to_rx;
        state.can_state = static_cast<uint8_t>(can_status.state);

        // ESP-IDF returns TWAI to STOPPED after bus-off recovery, so restart
        // it explicitly. Protocol handshakes are invalidated first to prevent
        // stale torque commands from being emitted as the peripheral returns.
        if (can_status.state == TWAI_STATE_BUS_OFF &&
            !g_bus_recovery_pending) {
            invalidate_all_controller_links("TWAI bus-off");
            const esp_err_t recovery_result = twai_initiate_recovery();
            if (recovery_result == ESP_OK) {
                g_bus_recovery_pending = true;
                Serial.println("[CAN] TWAI bus-off recovery started");
            } else {
                Serial.printf("[CAN] TWAI recovery start failed: %d\n",
                              static_cast<int>(recovery_result));
            }
        } else if (can_status.state == TWAI_STATE_STOPPED &&
                   g_bus_recovery_pending) {
            const esp_err_t restart_result = twai_start();
            if (restart_result == ESP_OK) {
                g_bus_recovery_pending = false;
                Serial.println(
                    "[CAN] TWAI restarted; awaiting controller handshakes");
            } else {
                Serial.printf("[CAN] TWAI restart failed: %d\n",
                              static_cast<int>(restart_result));
            }
        }
    }

    const uint32_t cluster_stale_ms =
        (uint32_t)realcar_cal::bringup::CLUSTER_COMMAND_STALE_MS;
    if (!fresh(state.cluster_cmd_last_rx_ms, cluster_stale_ms)) {
        state.cluster_cmd_alive = false;
        state.tv_enable_requested = false;
        state.regen_auto_requested = false;
        state.debug_requested = false;
        state.paddock_requested = false;
    }
    if (!fresh(state.bms_last_rx_ms, 5000U)) state.pack_data_valid = false;
    g_handshaked = realcar_cal::bringup::REQUIRE_BOTH_MOTOR_CONTROLLERS
        ? (g_handshaked_L && g_handshaked_R)
        : (g_handshaked_L || g_handshaked_R);
}

bool handshaked() { return g_handshaked; }
bool deadman_ok() { return (millis() - g_last_cmd_ms) < DEADMAN_MS; }
void note_command() { g_last_cmd_ms = millis(); }

} // namespace can_bus
