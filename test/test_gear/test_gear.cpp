#include <unity.h>
#include "modules/gear.h"

static GearCalib calib() { return {0, 2048, 4095, 450}; }

void test_nominal_positions(void) {
    TEST_ASSERT_TRUE(gear_classify(0, calib()) == Gear::Neutral);
    TEST_ASSERT_TRUE(gear_classify(2048, calib()) == Gear::Reverse);
    TEST_ASSERT_TRUE(gear_classify(4095, calib()) == Gear::Drive);
}

void test_outside_windows_fails_neutral(void) {
    TEST_ASSERT_TRUE(gear_classify(1000, calib()) == Gear::Neutral);
    TEST_ASSERT_TRUE(gear_classify(3000, calib()) == Gear::Neutral);
}

void test_filter_requires_stable_samples(void) {
    GearFilterState state;
    TEST_ASSERT_TRUE(gear_update(4095, calib(), 3, state) == Gear::Neutral);
    TEST_ASSERT_TRUE(gear_update(4095, calib(), 3, state) == Gear::Neutral);
    TEST_ASSERT_TRUE(gear_update(4095, calib(), 3, state) == Gear::Drive);
    TEST_ASSERT_TRUE(gear_update(2048, calib(), 3, state) == Gear::Drive);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_nominal_positions);
    RUN_TEST(test_outside_windows_fails_neutral);
    RUN_TEST(test_filter_requires_stable_samples);
    return UNITY_END();
}
