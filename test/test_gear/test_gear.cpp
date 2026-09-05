#include <unity.h>
#include "modules/gear.h"

static GearCalib calib() { return {0, 500, 2500, 0}; }

void test_contiguous_ranges(void) {
    TEST_ASSERT_TRUE(gear_classify(0, calib()) == Gear::Neutral);
    TEST_ASSERT_TRUE(gear_classify(499, calib()) == Gear::Neutral);
    TEST_ASSERT_TRUE(gear_classify(500, calib()) == Gear::Reverse);
    TEST_ASSERT_TRUE(gear_classify(2499, calib()) == Gear::Reverse);
    TEST_ASSERT_TRUE(gear_classify(2500, calib()) == Gear::Drive);
    TEST_ASSERT_TRUE(gear_classify(4095, calib()) == Gear::Drive);
}

void test_filter_requires_stable_samples(void) {
    GearFilterState state;
    TEST_ASSERT_TRUE(gear_update(3200, calib(), 3, state) == Gear::Neutral);
    TEST_ASSERT_TRUE(gear_update(3200, calib(), 3, state) == Gear::Neutral);
    TEST_ASSERT_TRUE(gear_update(3200, calib(), 3, state) == Gear::Drive);
    TEST_ASSERT_TRUE(gear_update(1500, calib(), 3, state) == Gear::Drive);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_contiguous_ranges);
    RUN_TEST(test_filter_requires_stable_samples);
    return UNITY_END();
}
