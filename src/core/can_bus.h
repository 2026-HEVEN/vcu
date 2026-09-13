// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
#include "can_protocol.h"
// [LOCKED] TWAI (ESP32 built-in CAN) driver + 50ms life-signal task.

namespace can_bus {
    void begin();           // install + start TWAI at 250 kbps
    void start_life_task(); // spawn high-priority 50ms TX task (life + torque frames)
    // Publish one control tick's final left/right command as a single unit.
    // The core-1 life task copies it whole, so the two motors can never be
    // commanded from different ticks. Call once per tick, AFTER the safety
    // verdict for that tick is final. See docs/M2_COMMAND_SNAPSHOT.md.
    void publish_motor_command(const MotorCommandSnapshot &snapshot);
    void poll_rx();         // drain RX queue into `state` (call from a scheduler task)
    void send_vehicle_speed(); // VCU -> Cluster/TMA-1 single speed telemetry
    void send_cluster_status(); // VCU -> Cluster gear/brake/HV state
    void send_sensor_telemetry(); // VCU -> TMA-1 steering + IMU telemetry
    bool handshaked();      // controller handshake completed
    bool deadman_ok();      // a fresh control command arrived within timeout
    void note_command();    // call when a new control command is produced
}
