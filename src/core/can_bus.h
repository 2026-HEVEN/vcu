// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include <stdint.h>
#include "safety_logic.h"
// [LOCKED] TWAI (ESP32 built-in CAN) driver + 50ms life-signal task.

namespace can_bus {
    void begin();           // install + start TWAI at 250 kbps
    void start_life_task(); // spawn high-priority 50ms TX task (life + torque frames)
    // Publish one control tick's final left/right command as a single unit.
    // The core-1 life task copies it whole, so the two motors can never be
    // commanded from different ticks. Call once per tick, AFTER the safety
    // verdict for that tick is final. See docs/M2_COMMAND_SNAPSHOT.md.
    void publish_motor_command(const MotorCommandSnapshot &snapshot);
    // Atomic pair, including queue outcomes. Queued is NOT controller receipt.
    MotorTxDiagnostics motor_tx_diagnostics();
    // Counts established controller links that were subsequently invalidated.
    // Initial boot before the first handshake is not a link loss.
    void link_drop_counts(uint32_t &left, uint32_t &right);
    void poll_rx();         // drain RX queue into `state` (call from a scheduler task)
    void send_vehicle_speed(); // VCU -> Cluster/TMA-1 single speed telemetry
    void send_cluster_status(); // VCU -> Cluster gear/brake/HV state
    void send_sensor_telemetry(); // VCU -> TMA-1 steering + IMU telemetry
    // 10ms 주기 호출. drive/motor는 매 틱(100Hz), TV 두 프레임은 격틱으로
    // 엇갈려 50Hz로 나간다 - 같은 틱에 4개가 몰리는 버스트를 피한다.
    void send_log_frames();
    // 1Hz. Amp 포화 통계 — 주행 중에는 시리얼 CLAMP 명령을 칠 수 없으므로
    // 이 경로가 유일한 관측 수단이다.
    void send_clamp_stats();
    void send_drive_diagnostics(); // 10Hz raw ADC + block + fault history
    bool rearm_controller_fault(); // explicit request, no deferred retry
    bool handshaked();      // controller handshake completed
    bool deadman_ok();      // a fresh control command arrived within timeout
    void note_command();    // call when a new control command is produced
}
