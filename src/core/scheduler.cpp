// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include <Arduino.h>
#include "scheduler_logic.h"
#include "core/wiring.h"   // provides g_tasks[] and G_TASK_COUNT

// [LOCKED] Runtime loop. The task table lives in app_wiring.cpp.
void scheduler_run() {
    scheduler_tick(g_tasks, G_TASK_COUNT, millis());
}
