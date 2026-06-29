#include <unity.h>
#include "modules/wheel_speed.h"

// 45 pulses/rev. 45 pulses in 100ms => 10 rev/s => 600 rpm.
void test_typical(void) {
    Rpm r = wheel_speed_compute({45, 100}, {45.0f});
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 600.0f, (float)r);
}
void test_zero_dt_safe(void) {  // never divide by zero
    Rpm r = wheel_speed_compute({10, 0}, {45.0f});
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)r);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_typical);
    RUN_TEST(test_zero_dt_safe);
    return UNITY_END();
}
