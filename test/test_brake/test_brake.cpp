#include <unity.h>
#include "modules/brake.h"

void test_released(void)      { BrakeOutput o = brake_compute({0});    TEST_ASSERT_FALSE(o.active); }
void test_pressed_active(void){ BrakeOutput o = brake_compute({4095}); TEST_ASSERT_TRUE(o.active);
                                TEST_ASSERT_EQUAL_FLOAT(100.0f, (float)o.pct); }

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_released);
    RUN_TEST(test_pressed_active);
    return UNITY_END();
}
