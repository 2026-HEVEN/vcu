// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#pragma once
// Prints the shared state over Serial so teammates can watch their
// compute() outputs live via `pio device monitor`. No storage, no extra deps.

void debug_update();

// One-shot service requests parsed from Serial. Commands are deliberately
// separated from the normal 5 Hz diagnostic output so app_wiring owns the
// safety-checked pulse state machine.
bool debug_consume_time_sync_arm_request();
bool debug_consume_time_sync_run_request();
bool debug_consume_time_sync_cancel_request();
