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
#include "modules/fixed_config.h"
#include "modules/motor_direction.h"
#include "modules/car_check_status.h"
#include "modules/tv/tv_config.h"

// [LOCKED] Bit layout of EZkontrol control frames follows
// EZkontrol-CANBUS-MCU-to-VCU.pdf and the reference 2026/Can_driver/CAN_DRIVER.ino.

// NOTE (deadman scope): note_command() is currently called every scheduler
// pass from app_wiring (torque_vectoring_update), so deadman_ok() effectively
// means "loop()/scheduler is still alive within 200ms", NOT "a fresh, valid
// command source exists". This is acceptable for the skeleton (it catches a
// hung scheduler). When CAN RX/handshake + throttle plausibility are
// implemented, gate note_command() on a genuinely valid command instead.
//
// NOTE (concurrency): the final command crosses cores only as one
// g_cmd_mux-guarded MotorCommandSnapshot, so both motors always follow the
// same tick and the same gear reading. See docs/M2_COMMAND_SNAPSHOT.md.

namespace {
    constexpr uint32_t DEADMAN_MS = 200;
    volatile uint32_t  g_last_cmd_ms = 0;
    volatile bool      g_handshaked  = false;
    volatile bool      g_handshaked_L = false;
    volatile bool      g_handshaked_R = false;
    volatile bool      g_reconnect_inhibit = false;
    volatile bool      g_reconnect_ramp_active = false;
    volatile uint32_t  g_reconnect_ramp_start_ms = 0U;
    volatile bool      g_feedback_recovery_expected_L = false;
    volatile bool      g_feedback_recovery_expected_R = false;
    uint32_t            g_handshake_reply_ms_L = 0U;
    uint32_t            g_handshake_reply_ms_R = 0U;
    bool                g_bus_recovery_pending = false;
    uint8_t            g_life = 0;
    uint8_t            g_status_life = 0;
    MotorTxDiagnostics g_tx_work{}; // life task owns counters and history
    RearmDwell g_fault_rearm_dwell{}; // RX scheduler only
    void latch_fault(uint8_t origin) {
        if (!state.controller_fault_latched) {
            state.fault_origin = 0;
            std::memset(state.first_fault_bytes, 0, sizeof(state.first_fault_bytes));
            state.fault_first_ms = millis();
        }
        if ((origin & 1U) && !(state.fault_origin & 1U)) {
            state.first_fault_bytes[0] = state.controller_fb2_L.error1;
            state.first_fault_bytes[1] = state.controller_fb2_L.error2;
            state.first_fault_bytes[2] = state.controller_fb2_L.error3;
        }
        if ((origin & 2U) && !(state.fault_origin & 2U)) {
            state.first_fault_bytes[3] = state.controller_fb2_R.error1;
            state.first_fault_bytes[4] = state.controller_fb2_R.error2;
            state.first_fault_bytes[5] = state.controller_fb2_R.error3;
        }
        state.fault_origin |= origin;
        state.controller_fault_latched = true;
        state.fault_rearm_ready = false;
        g_fault_rearm_dwell.tracking = false;
    }
    bool rearm_conditions_ok() {
        return g_handshaked_L && g_handshaked_R &&
            state.controller_feedback_fresh_L && state.controller_feedback_fresh_R &&
            !state.controller_fb2_L.any_fault() && !state.controller_fb2_R.any_fault() &&
            !state.controller_fb2_L.speed_mode && !state.controller_fb2_R.speed_mode &&
            state.throttle_signal_valid && (float)state.throttle_pct == 0.0f &&
            std::abs(state.controller_fb1_L.motor_speed_rpm) <= 50 &&
            std::abs(state.controller_fb1_R.motor_speed_rpm) <= 50 &&
            std::isfinite(state.controller_fb1_L.phase_current_a) &&
            std::isfinite(state.controller_fb1_R.phase_current_a) &&
            std::fabs(state.controller_fb1_L.phase_current_a) <= fixed_config::runtime::PHASE_CURRENT_HARD_CUTOFF_A &&
            std::fabs(state.controller_fb1_R.phase_current_a) <= fixed_config::runtime::PHASE_CURRENT_HARD_CUTOFF_A &&
            state.controller_fb2_L.controller_temp_c < realcar_cal::bringup::CONTROLLER_DERATE_START_C &&
            state.controller_fb2_R.controller_temp_c < realcar_cal::bringup::CONTROLLER_DERATE_START_C &&
            state.controller_fb2_L.motor_temp_c < realcar_cal::bringup::MOTOR_DERATE_START_C &&
            state.controller_fb2_R.motor_temp_c < realcar_cal::bringup::MOTOR_DERATE_START_C &&
            !state.component_test_active && !state.component_test_normal_inhibit &&
            !state.time_sync_armed && !state.time_sync_active;
    }
    portMUX_TYPE         g_link_diag_mux = portMUX_INITIALIZER_UNLOCKED;
    uint32_t             g_link_drops_L = 0U; // guarded by g_link_diag_mux
    uint32_t             g_link_drops_R = 0U;

    // The only command data shared between the cores.
    portMUX_TYPE         g_cmd_mux = portMUX_INITIALIZER_UNLOCKED;
    MotorCommandSnapshot g_cmd_snapshot{};   // guarded by g_cmd_mux
    MotorTxDiagnostics g_tx_diagnostics{};   // also guarded by g_cmd_mux

    // Never transmit inside the critical section: twai_transmit() blocks.
    MotorCommandSnapshot copy_motor_command() {
        MotorCommandSnapshot out;
        portENTER_CRITICAL(&g_cmd_mux);
        out = g_cmd_snapshot;
        portEXIT_CRITICAL(&g_cmd_mux);
        return out;
    }

    void invalidate_controller_link(bool left, const char *reason) {
        // The RX scheduler and TX task can both invalidate a link. Count the
        // established -> invalid transition exactly once across the two cores.
        portENTER_CRITICAL(&g_link_diag_mux);
        const bool was_handshaked = left ? g_handshaked_L : g_handshaked_R;
        if (left) {
            g_handshaked_L = false;
            if (was_handshaked) ++g_link_drops_L;
        } else {
            g_handshaked_R = false;
            if (was_handshaked) ++g_link_drops_R;
        }
        portEXIT_CRITICAL(&g_link_diag_mux);
        if (left) {
            state.controller_handshaked_L = false;
            state.controller_feedback_fresh_L = false;
            state.controller_fb1_last_ms_L = 0U;
            state.controller_fb2_last_ms_L = 0U;
            g_feedback_recovery_expected_L = true;
        } else {
            state.controller_handshaked_R = false;
            state.controller_feedback_fresh_R = false;
            state.controller_fb1_last_ms_R = 0U;
            state.controller_fb2_last_ms_R = 0U;
            g_feedback_recovery_expected_R = true;
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

    bool transmit_ext(uint32_t id, const uint8_t data[8], TickType_t wait_ticks=pdMS_TO_TICKS(5)) {
        twai_message_t m = {};
        m.identifier = id;
        m.extd = 1;
        m.data_length_code = 8;
        for (int i = 0; i < 8; ++i) m.data[i] = data[i];
        return twai_transmit(&m, wait_ticks) == ESP_OK;
    }

    // No `state` access: target_rpm is decided once per tick from the
    // snapshot's single gear field. Returns false if the frame never queued.
    bool send_torque(uint32_t id, float amps, int target_rpm,
                     bool running, uint8_t life) {
        uint8_t data[8];
        encode_motor_control(amps, target_rpm, running, life, data);

        twai_message_t m = {};
        m.identifier = id; m.extd = 1; m.data_length_code = 8;
        for (int i = 0; i < 8; ++i) m.data[i] = data[i];
        return twai_transmit(&m, pdMS_TO_TICKS(5)) == ESP_OK;
    }

    // A frame we could not even queue is not a command. Repeated failure goes
    // to the existing link-loss policy: both motors 0 A, reconnect ramp back.
    void note_tx_result(bool left, const MotorTxSideDiagnostics &side, unsigned &telemetry) {
        telemetry = side.consecutive_failures;
        if (side.result == MotorTxResult::Failed &&
            side.consecutive_failures == fixed_config::runtime::MOTOR_TX_FAIL_LIMIT) {
            invalidate_controller_link(left, "tx failed");
        }
    }

    void life_task(void *) {
        const TickType_t period = pdMS_TO_TICKS(
            fixed_config::runtime::MOTOR_COMMAND_PERIOD_MS);
        TickType_t next = xTaskGetTickCount();
        for (;;) {
            const uint32_t now = millis();
            const bool scheduler_alive = (now - g_last_cmd_ms < DEADMAN_MS);
            if (g_reconnect_inhibit) {
                const bool protocol_ready =
                    fixed_config::runtime::REQUIRE_BOTH_MOTOR_CONTROLLERS
                        ? (g_handshaked_L && g_handshaked_R)
                        : (g_handshaked_L || g_handshaked_R);
                const bool feedback_ready =
                    fixed_config::runtime::REQUIRE_BOTH_MOTOR_CONTROLLERS
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
                        fixed_config::runtime::THROTTLE_ARM_MAX_PCT) {
                    if (state.component_test_release_ticks <
                        fixed_config::bench::COMPONENT_TEST_RELEASE_TICKS) {
                        ++state.component_test_release_ticks;
                    }
                    if (state.component_test_release_ticks >=
                        fixed_config::bench::COMPONENT_TEST_RELEASE_TICKS) {
                        state.component_test_normal_inhibit = false;
                    }
                } else {
                    state.component_test_release_ticks = 0U;
                }
            }
            // 1) 좌우 전류·기어·구동 허용 상태를 한 묶음으로 복사한다.
            //    이번 송신에서는 이 복사본만 사용한다.
            const MotorCommandSnapshot snap = copy_motor_command();
            const uint32_t command_heartbeat_ms = g_last_cmd_ms;
            // 2) 명령을 받은 다음 시계를 읽고, 명령이 얼마나 오래됐는지 계산한다.
            //    루프 시작의 now를 쓰지 말 것: 그 이후 새 명령이 게시될 수 있다.
            //    예: 옛 now=1000, 게시=1001이면 unsigned 차이는 4294967295가 된다.
            const uint32_t checked_at_ms = millis();
            const uint32_t snapshot_age_ms = checked_at_ms - snap.published_ms;
            // 3) 제어 루프와 명령의 유효기간을 검사한 뒤 송신값을 결정한다.
            MotorCommandGates gates;
            gates.scheduler_alive = checked_at_ms - command_heartbeat_ms < DEADMAN_MS;
            gates.reconnect_inhibit = g_reconnect_inhibit;
            gates.component_test_inhibit = state.component_test_normal_inhibit;
            gates.snapshot_fresh = motor_snapshot_fresh(snap, checked_at_ms,
                fixed_config::runtime::MOTOR_COMMAND_SNAPSHOT_MAX_AGE_MS);
            gates.reconnect_ramp_scale = 1.0f;   // ramp applied after the test branch
            const MotorFrameCommand resolved = motor_command_resolve(snap, gates);

            bool normal_allow = resolved.normal_allow;
            float l = resolved.left_a;
            float r = resolved.right_a;
            bool run_l = resolved.run_L;
            bool run_r = resolved.run_R;
            int rpm_l = resolved.target_rpm_L;
            int rpm_r = resolved.target_rpm_R;

            // Branch-only component test path. While a test is active it has
            // exclusive ownership of both command outputs: the unselected
            // motor is explicitly HALTED and normal throttle cannot mix in.
            if (state.component_test_active) {
                normal_allow = false;
                l = 0.0f;
                r = 0.0f;
                run_l = false;
                run_r = false;
                rpm_l = 0;
                rpm_r = 0;

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
                            fixed_config::runtime::THROTTLE_ARM_MAX_PCT &&
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
                            fixed_config::runtime::PHASE_CURRENT_HARD_CUTOFF_A);
                    const bool right_ok = !state.component_test_right ||
                        (g_handshaked_R && state.controller_feedback_fresh_R &&
                         !state.controller_fb2_R.any_fault() &&
                         state.controller_fb2_R.controller_temp_c <
                            realcar_cal::bringup::CONTROLLER_CUTOFF_C &&
                         state.controller_fb2_R.motor_temp_c <
                            realcar_cal::bringup::MOTOR_CUTOFF_C &&
                         !state.controller_fb2_R.speed_mode &&
                         std::fabs(state.controller_fb1_R.phase_current_a) <
                            fixed_config::runtime::PHASE_CURRENT_HARD_CUTOFF_A);
                    if (!common_ok || !left_ok || !right_ok) {
                        state.component_test_active = false;
                        ++state.component_test_aborted_count;
                    } else {
                        const float test_a = std::fmax(0.0f, std::fmin(
                            state.component_test_current_a,
                            fixed_config::bench::COMPONENT_TEST_CURRENT_MAX_PER_MOTOR_A));
                        // common_ok above already requires Gear::Drive, and
                        // test_a is clamped non-negative: forward on both sides.
                        if (state.component_test_left) {
                            l = test_a;
                            run_l = true;
                            rpm_l = DRIVE_TARGET_SPEED_RPM;
                        }
                        if (state.component_test_right) {
                            r = test_a;
                            run_r = true;
                            rpm_r = DRIVE_TARGET_SPEED_RPM;
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
                    fixed_config::runtime::MOTOR_RECONNECT_RAMP_MS;
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
            state.motor_command_seq = snap.seq;

            // Queue both frames back to back, no work in between.
            const auto tx_l = !g_handshaked_L ? MotorTxResult::Skipped :
                (send_torque(CAN_ID_TORQUE_L, l, rpm_l, run_l, g_life)
                    ? MotorTxResult::Queued : MotorTxResult::Failed);
            const auto tx_r = !g_handshaked_R ? MotorTxResult::Skipped :
                (send_torque(CAN_ID_TORQUE_R, r, rpm_r, run_r, g_life)
                    ? MotorTxResult::Queued : MotorTxResult::Failed);
            const uint32_t tx_done_ms = millis();
            motor_tx_record(g_tx_work.left, tx_l, l, rpm_l, run_l, snap.seq, tx_done_ms);
            motor_tx_record(g_tx_work.right, tx_r, r, rpm_r, run_r, snap.seq, tx_done_ms);
            g_tx_work.seq = snap.seq;
            g_tx_work.requested.left_a = l; g_tx_work.requested.right_a = r;
            g_tx_work.requested.target_rpm_L = rpm_l; g_tx_work.requested.target_rpm_R = rpm_r;
            g_tx_work.requested.run_L = run_l; g_tx_work.requested.run_R = run_r;
            g_tx_work.requested.normal_allow = normal_allow;
            g_tx_work.requested.block_reasons = resolved.block_reasons |
                (state.component_test_active ? BLOCK_TEST : 0);
            g_tx_work.snapshot_age_ms = snapshot_age_ms;
            g_tx_work.snapshot_fresh = gates.snapshot_fresh;
            if (!gates.snapshot_fresh && g_tx_work.stale_total != UINT32_MAX)
                ++g_tx_work.stale_total;
            portENTER_CRITICAL(&g_cmd_mux);
            g_tx_diagnostics = g_tx_work;
            portEXIT_CRITICAL(&g_cmd_mux);
            note_tx_result(true, g_tx_work.left, state.can_tx_fail_count_L);
            note_tx_result(false, g_tx_work.right, state.can_tx_fail_count_R);

            g_life++;   // after both sends: the pair shares one life value
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
    g.rx_queue_len = fixed_config::runtime::CAN_RX_QUEUE_LENGTH;
    g.tx_queue_len = 16; // six display frames + two motor frames can coincide
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_driver_install(&g, &t, &f);
    twai_start();
}

void publish_motor_command(const MotorCommandSnapshot &snapshot) {
    // Critical section holds a single struct copy and nothing else.
    portENTER_CRITICAL(&g_cmd_mux);
    g_cmd_snapshot = snapshot;
    portEXIT_CRITICAL(&g_cmd_mux);
}

MotorTxDiagnostics motor_tx_diagnostics() {
    MotorTxDiagnostics out;
    portENTER_CRITICAL(&g_cmd_mux);
    out = g_tx_diagnostics;
    portEXIT_CRITICAL(&g_cmd_mux);
    return out;
}

bool rearm_controller_fault() {
    if (!state.controller_fault_latched || !state.fault_rearm_ready ||
        !rearm_conditions_ok()) return false;
    safety_require_rearm();
    state.controller_fault_latched = false;
    state.fault_rearm_ready = false;
    g_fault_rearm_dwell.tracking = false;
    ++state.fault_rearm_count;
    return true;
}

void send_drive_diagnostics() {
    uint8_t d[8]{};
    const auto put_u16 = [&](unsigned k, uint16_t v) {
        d[k] = (uint8_t)v; d[k+1] = (uint8_t)(v >> 8);
    };
    const auto send = [&](uint32_t id) {
        const bool ok = transmit_ext(id, d, 0);
        if (!ok) ++state.drive_diagnostic_tx_drops;
        return ok;
    };
    put_u16(0, (uint16_t)state.throttle_raw_adc);
    put_u16(2, state.throttle_window_min);
    put_u16(4, state.throttle_last_invalid_raw);
    put_u16(6, state.throttle_invalid_samples);
    if (send(CAN_ID_VCU_LOG_THROTTLE))
        state.throttle_window_min = (uint16_t)state.throttle_raw_adc;
    put_u16(0, state.diagnostic_block_reasons);
    put_u16(2, state.first_block_reasons);
    put_u16(4, state.block_event_count);
    put_u16(6, (state.drive_slew_limited ? 1 : 0) | (state.thermal_limited ? 2 : 0) |
        (state.power_limited ? 4 : 0) | (state.paddock_current_limited ? 8 : 0) |
        (state.paddock_active ? 16 : 0) | (state.fault_rearm_ready ? 32 : 0));
    send(CAN_ID_VCU_LOG_BLOCK);
    for (unsigned i = 0; i < 6; ++i) d[i] = state.first_fault_bytes[i];
    d[6] = state.fault_origin;
    d[7] = (state.controller_fault_latched ? 1 : 0) |
        (state.fault_rearm_ready ? 2 : 0);
    send(CAN_ID_VCU_LOG_FAULT);
}

void start_life_task() {
    // High priority on core 1. Arduino loop may also use core 1; do not assume
    // different cores when reasoning about yields or scheduling latency.
    xTaskCreatePinnedToCore(life_task, "can_life", 4096, nullptr, 20, nullptr, 1);
}


void send_log_frames() {
    static uint8_t tick = 0;
    static uint8_t log_life = 0;
    uint8_t data[8];
    // 대기시간 0. 로깅이 제어 스케줄러를 막아서는 안 된다 - 버스가 붐비면
    // 그 프레임은 버리고 다음 틱에 다시 보낸다(100Hz라 손실이 무해하다).
    const auto send = [&](uint32_t id) {
        if (!transmit_ext(id, data, 0)) ++state.sensor_telemetry_tx_drops;
    };

    // 100Hz - 명령 전류와 실제 상전류. 이 둘의 비율이 컨트롤러가 명령을
    // 그대로 흘리는지 보여준다.
    const auto tx = motor_tx_diagnostics();
    encode_vcu_log_drive(tx.requested.left_a,
                         tx.requested.right_a,
                         state.controller_fb1_L.phase_current_a,
                         state.controller_fb1_R.phase_current_a, data);
    send(CAN_ID_VCU_LOG_DRIVE);

    // 100Hz - 회전수와 모선전류
    encode_vcu_log_motor((int)state.controller_fb1_L.motor_speed_rpm,
                         (int)state.controller_fb1_R.motor_speed_rpm,
                         state.controller_fb1_L.bus_current_a,
                         state.controller_fb1_R.bus_current_a, data);
    send(CAN_ID_VCU_LOG_MOTOR);

    if ((tick & 1u) == 0u) {
        // 50Hz(짝수 틱) - 요 제어 경로. requested_torque는 TV 출력이고
        // can_commanded_current는 drive_supervisor를 거친 값이라, 둘의 차이가
        // 파워/상승률/열 제한이 깎은 양이다.
        encode_vcu_log_tv_yaw(state.desired_yaw_rate, state.yaw_moment,
                              (float)state.requested_torque_L,
                              (float)state.requested_torque_R, data);
        send(CAN_ID_VCU_LOG_TV_YAW);
    } else {
        // 50Hz(홀수 틱) - 하중과 트랙션 한계, 게이트 사유
        const TVGate &g = state.tv_gate;
        const uint8_t gate_bits =
            (uint8_t)((g.active ? 0x01u : 0u) |
                      (g.driver_switch_on ? 0x02u : 0u) |
                      (g.gains_enabled ? 0x04u : 0u) |
                      (g.imu_valid ? 0x08u : 0u) |
                      (g.speed_valid ? 0x10u : 0u) |
                      (g.speed_above_min ? 0x20u : 0u));
        encode_vcu_log_tv_load(state.fz_L, state.fz_R,
                               state.max_torque_L, state.max_torque_R,
                               gate_bits, log_life++, data);
        send(CAN_ID_VCU_LOG_TV_LOAD);
    }
    ++tick;
}

void send_clamp_stats() {
    const ClampStats s = Amp::clamp_stats();
    uint8_t data[8];
    encode_vcu_log_clamp(s.high_count, s.low_count,
                         s.high_worst.raw, s.low_worst.raw, data);
    // 여기서도 대기시간 0. 진단이 제어를 막아서는 안 된다.
    if (!transmit_ext(CAN_ID_VCU_LOG_CLAMP, data, 0))
        ++state.sensor_telemetry_tx_drops;
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
    static uint8_t life = 0;
    uint8_t data[8];
    const auto send = [&](uint32_t id) {
        // Display data must not add four 5 ms waits to the control scheduler.
        if (!transmit_ext(id, data, 0)) ++state.sensor_telemetry_tx_drops;
    };
    state.steering_telemetry.life = life;
    state.imu_telemetry.life = life;
    car_check::encode_steering(state.steering_telemetry, data);
    send(car_check::STEERING_ID);
    car_check::encode_imu(state.imu_telemetry, data);
    send(car_check::IMU_ID);
    car_check::encode_wheels(state.wheel_telemetry, data);
    send(car_check::WHEELS_ID);

    const auto tx_status = motor_tx_diagnostics();
    const bool output_allowed = tx_status.seq != 0 && tx_status.snapshot_fresh &&
        tx_status.requested.normal_allow && torque_allowed() && deadman_ok() &&
        state.throttle_signal_valid && !g_reconnect_inhibit &&
        !state.component_test_normal_inhibit && state.propulsion_direction_armed &&
        state.controller_feedback_fresh && !state.controller_fault_latched &&
        (state.gear == Gear::Drive || state.gear == Gear::Reverse);
    // 차단 사유를 여기서 재계산하지 않는다. 판정은 tv_gate_evaluate()가 이미
    // 했고, app_wiring이 state.tv_gate에 넣어둔 그 결과만 그대로 보고한다.
    const TVGate &tv_gate = state.tv_gate;
    const CarCheckStatusInput in {
        state.tv_enable_requested, state.regen_auto_requested,
        state.paddock_requested, state.paddock_active, state.cluster_cmd_alive,
        state.tv_pipeline_active, tv_gate.gains_enabled, tv_gate.imu_valid,
        tv_gate.speed_valid,
        tv_gate.speed_above_min,
        output_allowed, state.component_test_active || state.time_sync_active,
        realcar_cal::bringup::REGEN_HARDWARE_VALIDATED,
        realcar_cal::bringup::BRAKE_SENSOR_INSTALLED, state.pack_data_valid,
        state.longitudinal_regen_demand,
        state.pack_soc, (float)state.torque_L, (float)state.torque_R,
        state.gear==Gear::Drive ? 1 : (state.gear==Gear::Reverse ? -1 : 0)
    };
    car_check::Control status = car_check_status_compute(in);
    status.life = life++;
    car_check::encode_control(status, data);
    send(car_check::CONTROL_ID);
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
                    portENTER_CRITICAL(&g_link_diag_mux);
                    g_handshaked_L = true;
                    portEXIT_CRITICAL(&g_link_diag_mux);
                    state.controller_handshaked_L = true;
                    g_feedback_recovery_expected_L = false;
                    g_handshake_reply_ms_L = millis();
                } else {
                    portENTER_CRITICAL(&g_link_diag_mux);
                    g_handshaked_R = true;
                    portEXIT_CRITICAL(&g_link_diag_mux);
                    state.controller_handshaked_R = true;
                    g_feedback_recovery_expected_R = false;
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
                fixed_config::runtime::PHASE_CURRENT_HARD_CUTOFF_A) {
                latch_fault(4U);
            }
            continue;
        }
        if (m.data_length_code == 8 && m.identifier == CAN_ID_FB1_R) {
            state.controller_fb1_R = decode_controller_feedback_part1(m.data);
            state.controller_fb1_last_ms_R = now;
            if (std::fabs(state.controller_fb1_R.phase_current_a) >
                fixed_config::runtime::PHASE_CURRENT_HARD_CUTOFF_A) {
                latch_fault(8U);
            }
            continue;
        }
        if (m.data_length_code == 8 && m.identifier == CAN_ID_FB2_L) {
            state.controller_fb2_L = decode_controller_feedback_part2(m.data);
            state.controller_fb2_last_ms_L = now;
            if (state.controller_fb2_L.any_fault()) latch_fault(1U);
            continue;
        }
        if (m.data_length_code == 8 && m.identifier == CAN_ID_FB2_R) {
            state.controller_fb2_R = decode_controller_feedback_part2(m.data);
            state.controller_fb2_last_ms_R = now;
            if (state.controller_fb2_R.any_fault()) latch_fault(2U);
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
        (uint32_t)fixed_config::runtime::CONTROLLER_FEEDBACK_STALE_MS;
    state.controller_feedback_fresh_L =
        fresh(state.controller_fb1_last_ms_L, feedback_stale_ms) &&
        fresh(state.controller_fb2_last_ms_L, feedback_stale_ms);
    state.controller_feedback_fresh_R =
        fresh(state.controller_fb1_last_ms_R, feedback_stale_ms) &&
        fresh(state.controller_fb2_last_ms_R, feedback_stale_ms);
    state.controller_feedback_fresh =
        state.controller_feedback_fresh_L &&
        state.controller_feedback_fresh_R;

    // After a real link loss, some controller/firmware combinations resume
    // normal Part I/II feedback without returning to the documented 0x55
    // startup probe.  In that case the VCU used to remain stuck at hs=0 even
    // though both fresh feedback frames proved that the controller-side CAN
    // session was alive.  Accept that evidence only for a link which was
    // previously handshaked and then explicitly invalidated; initial startup
    // still requires the normal 0x55/0xAA exchange.  The global reconnect
    // inhibit remains set until both sides are ready, after which the existing
    // one-second torque ramp performs the controlled recovery.
    if (!g_handshaked_L && g_feedback_recovery_expected_L &&
        state.controller_feedback_fresh_L) {
        portENTER_CRITICAL(&g_link_diag_mux);
        g_handshaked_L = true;
        portEXIT_CRITICAL(&g_link_diag_mux);
        state.controller_handshaked_L = true;
        g_feedback_recovery_expected_L = false;
        g_handshake_reply_ms_L = now;
        Serial.println(
            "[CAN] controller L restored from fresh Part I/II feedback");
    }
    if (!g_handshaked_R && g_feedback_recovery_expected_R &&
        state.controller_feedback_fresh_R) {
        portENTER_CRITICAL(&g_link_diag_mux);
        g_handshaked_R = true;
        portEXIT_CRITICAL(&g_link_diag_mux);
        state.controller_handshaked_R = true;
        g_feedback_recovery_expected_R = false;
        g_handshake_reply_ms_R = now;
        Serial.println(
            "[CAN] controller R restored from fresh Part I/II feedback");
    }

    // The 250 ms freshness check above removes torque immediately. If either
    // required feedback part is still absent at the longer timeout, stop that
    // side's normal command frames. The controller can then enter its
    // documented timeout path and issue a fresh 0x55 handshake probe.
    const uint32_t rehandshake_timeout_ms =
        fixed_config::runtime::CONTROLLER_REHANDSHAKE_TIMEOUT_MS;
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
        (uint32_t)fixed_config::runtime::CLUSTER_COMMAND_STALE_MS;
    if (!fresh(state.cluster_cmd_last_rx_ms, cluster_stale_ms)) {
        state.cluster_cmd_alive = false;
        state.tv_enable_requested = false;
        state.regen_auto_requested = false;
        state.debug_requested = false;
        state.paddock_requested = false;
    }
    if (!fresh(state.bms_last_rx_ms, 5000U)) state.pack_data_valid = false;
    g_handshaked = fixed_config::runtime::REQUIRE_BOTH_MOTOR_CONTROLLERS
        ? (g_handshaked_L && g_handshaked_R)
        : (g_handshaked_L || g_handshaked_R);
    state.fault_rearm_ready = fault_rearm_dwell(
        state.controller_fault_latched && rearm_conditions_ok(), millis(), g_fault_rearm_dwell);
}

bool handshaked() { return g_handshaked; }
void link_drop_counts(uint32_t &left, uint32_t &right) {
    portENTER_CRITICAL(&g_link_diag_mux);
    left = g_link_drops_L;
    right = g_link_drops_R;
    portEXIT_CRITICAL(&g_link_diag_mux);
}
bool deadman_ok() { return (millis() - g_last_cmd_ms) < DEADMAN_MS; }
void note_command() { g_last_cmd_ms = millis(); }

} // namespace can_bus
