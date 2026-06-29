// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "scheduler_logic.h"

bool task_is_due(const Task &t, uint32_t now) {
    return (now - t.last_run) >= t.period_ms;
}

void scheduler_tick(Task *tasks, int n, uint32_t now) {
    for (int i = 0; i < n; i++) {
        if (task_is_due(tasks[i], now)) {
            tasks[i].update();
            tasks[i].last_run = now;
        }
    }
}
