#pragma once
#include "scheduler_logic.h"
// [LOCKED] Exposes the task table + init to scheduler.cpp / main.cpp.

extern Task g_tasks[];
extern const int G_TASK_COUNT;

void modules_init();   // bring up drivers + CAN
void safety_update();  // defined in core/safety.cpp
