// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "core/can_bus.h"
#include <Arduino.h>
#include "driver/twai.h"
#include "can_protocol.h"
#include "state.h"
#include "safety_logic.h"   // torque_allowed()
#include "core/board_pins.h"

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
    volatile uint32_t  g_last_cmd_ms = 0;
    volatile bool      g_handshaked  = false;
    uint8_t            g_life = 0;

    void transmit_ext(uint32_t id, const uint8_t data[8]) {
        twai_message_t m = {};
        m.identifier = id;
        m.extd = 1;
        m.data_length_code = 8;
        for (int i = 0; i < 8; ++i) m.data[i] = data[i];
        twai_transmit(&m, pdMS_TO_TICKS(5));
    }

    void send_torque(uint32_t id, float amps) {
        uint16_t raw = torque_to_raw(amps);
        twai_message_t m = {};
        m.identifier = id; m.extd = 1; m.data_length_code = 8;
        m.data[0] = raw & 0xFF; m.data[1] = (raw >> 8) & 0xFF;
        m.data[7] = g_life;                 // life counter byte (per EZkontrol spec)
        twai_transmit(&m, pdMS_TO_TICKS(5));
    }

    void life_task(void *) {
        const TickType_t period = pdMS_TO_TICKS(50);
        TickType_t next = xTaskGetTickCount();
        for (;;) {
            bool allow = torque_allowed() &&
                         (millis() - g_last_cmd_ms < DEADMAN_MS);
            float l = allow ? (float)state.torque_L : 0.0f;
            float r = allow ? (float)state.torque_R : 0.0f;
            send_torque(CAN_ID_TORQUE_L, l);
            send_torque(CAN_ID_TORQUE_R, r);
            g_life++;
            vTaskDelayUntil(&next, period);   // exact 50ms cadence
        }
    }
}

namespace can_bus {

void begin() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        static_cast<gpio_num_t>(board_pins::CAN_TX),
        static_cast<gpio_num_t>(board_pins::CAN_RX),
        TWAI_MODE_NORMAL);
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

void poll_rx() {
    twai_message_t m;
    while (twai_receive(&m, 0) == ESP_OK) {
        // TODO(core): parse controller feedback (speed/temp/err) + handshake
        // (8x 0x55 request -> reply 0x55) into `state`. Set g_handshaked on success.
        (void)m;
    }
}

bool handshaked() { return g_handshaked; }
bool deadman_ok() { return (millis() - g_last_cmd_ms) < DEADMAN_MS; }
void note_command() { g_last_cmd_ms = millis(); }

} // namespace can_bus
