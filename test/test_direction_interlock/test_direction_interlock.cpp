#include <unity.h>
#include "modules/direction_interlock.h"

void test_neutral_is_never_enabled() {
    DirectionInterlockState state{};
    const auto out = direction_interlock_update(Gear::Neutral, true, true, 3, state);
    TEST_ASSERT_FALSE(out.propulsion_enabled);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.command_sign);
}

void test_drive_arms_positive_after_release_delay() {
    DirectionInterlockState state{};
    TEST_ASSERT_FALSE(direction_interlock_update(Gear::Drive, true, true, 3, state).propulsion_enabled);
    TEST_ASSERT_FALSE(direction_interlock_update(Gear::Drive, true, true, 3, state).propulsion_enabled);
    const auto out = direction_interlock_update(Gear::Drive, true, true, 3, state);
    TEST_ASSERT_TRUE(out.propulsion_enabled);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out.command_sign);
}

void test_reverse_arms_negative_after_release_delay() {
    DirectionInterlockState state{};
    direction_interlock_update(Gear::Reverse, true, true, 2, state);
    const auto out = direction_interlock_update(Gear::Reverse, true, true, 2, state);
    TEST_ASSERT_TRUE(out.propulsion_enabled);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, out.command_sign);
}

void test_direction_change_disarms_until_released_again() {
    DirectionInterlockState state{};
    direction_interlock_update(Gear::Drive, true, true, 2, state);
    TEST_ASSERT_TRUE(direction_interlock_update(Gear::Drive, true, true, 2, state).propulsion_enabled);

    TEST_ASSERT_FALSE(direction_interlock_update(Gear::Reverse, false, true, 2, state).propulsion_enabled);
    TEST_ASSERT_FALSE(direction_interlock_update(Gear::Reverse, true, true, 2, state).propulsion_enabled);
    const auto out = direction_interlock_update(Gear::Reverse, true, true, 2, state);
    TEST_ASSERT_TRUE(out.propulsion_enabled);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, out.command_sign);
}

void test_motion_or_throttle_resets_pending_arm() {
    DirectionInterlockState state{};
    direction_interlock_update(Gear::Drive, true, true, 2, state);
    TEST_ASSERT_FALSE(direction_interlock_update(Gear::Drive, true, false, 2, state).propulsion_enabled);
    TEST_ASSERT_FALSE(direction_interlock_update(Gear::Drive, false, true, 2, state).propulsion_enabled);
    TEST_ASSERT_FALSE(direction_interlock_update(Gear::Drive, true, true, 2, state).propulsion_enabled);
    TEST_ASSERT_TRUE(direction_interlock_update(Gear::Drive, true, true, 2, state).propulsion_enabled);
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_neutral_is_never_enabled);
    RUN_TEST(test_drive_arms_positive_after_release_delay);
    RUN_TEST(test_reverse_arms_negative_after_release_delay);
    RUN_TEST(test_direction_change_disarms_until_released_again);
    RUN_TEST(test_motion_or_throttle_resets_pending_arm);
    return UNITY_END();
}

