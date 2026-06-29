#pragma once
#include <cstdint>
// [LOCKED] Cooperative scheduler. Pure decision logic is host-tested.

struct Task {
    void (*update)();
    uint32_t period_ms;
    uint32_t last_run;
};

bool task_is_due(const Task &t, uint32_t now);
void scheduler_tick(Task *tasks, int n, uint32_t now);

void scheduler_run();   // runtime loop body, defined in core/scheduler.cpp
