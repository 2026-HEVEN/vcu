#include <unity.h>
#include "scheduler_logic.h"

static int calls = 0;
static void bump() { calls++; }

void test_due_after_period(void) {
    Task t{ bump, 10, 0 };
    TEST_ASSERT_FALSE(task_is_due(t, 5));
    TEST_ASSERT_TRUE(task_is_due(t, 10));
}
void test_tick_runs_and_advances(void) {
    calls = 0;
    Task tasks[] = { { bump, 10, 0 } };
    scheduler_tick(tasks, 1, 10);   // due -> runs
    scheduler_tick(tasks, 1, 12);   // not due yet
    TEST_ASSERT_EQUAL_INT(1, calls);
    TEST_ASSERT_EQUAL_UINT32(10, tasks[0].last_run);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_due_after_period);
    RUN_TEST(test_tick_runs_and_advances);
    return UNITY_END();
}
