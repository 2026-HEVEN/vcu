// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
// [LOCKED] TWAI (ESP32 built-in CAN) driver + 50ms life-signal task.

namespace can_bus {
    void begin();           // install + start TWAI at 250 kbps
    void start_life_task(); // spawn high-priority 50ms TX task (life + torque frames)
    void poll_rx();         // drain RX queue into `state` (call from a scheduler task)
    void send_wheel_speeds(); // VCU -> Cluster WSS RPM telemetry
    void send_vehicle_speed(); // VCU -> TMA-1/Cluster single speed telemetry
    bool handshaked();      // controller handshake completed
    bool deadman_ok();      // a fresh control command arrived within timeout
    void note_command();    // call when a new control command is produced
}
