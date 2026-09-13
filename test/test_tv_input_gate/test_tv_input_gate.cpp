#include <unity.h>
#include "modules/tv/input_gate.h"

static TVInputGateInput healthy() {
    // switch on, IMU frame fresh, rate-of-turn fresh, steering valid
    return TVInputGateInput{ true, true, true, true, 10.0f };
}

void test_all_inputs_healthy_enables_tv(void) {
    TVInputGateState s{};
    TEST_ASSERT_TRUE(tv_input_gate_compute(healthy(), s).tv_enable);
}

void test_dash_switch_off_blocks_tv(void) {
    TVInputGateState s{};
    TVInputGateInput in = healthy();
    in.tv_requested = false;
    TEST_ASSERT_FALSE(tv_input_gate_compute(in, s).tv_enable);
}

void test_stale_imu_frames_block_tv(void) {
    TVInputGateState s{};
    TVInputGateInput in = healthy();
    in.imu_frame_fresh = false;
    TEST_ASSERT_FALSE(tv_input_gate_compute(in, s).tv_enable);
}

void test_frames_without_rate_of_turn_block_tv(void) {
    // MTData2 frames keep arriving, but the rate-of-turn group is missing,
    // so yaw_rate would be frozen at its last value.
    TVInputGateState s{};
    TVInputGateInput in = healthy();
    in.imu_frame_fresh = true;
    in.yaw_sample_fresh = false;
    TEST_ASSERT_FALSE(tv_input_gate_compute(in, s).tv_enable);
}

void test_invalid_steering_blocks_tv(void) {
    // e.g. sensor not installed or wiper pinned to a rail
    TVInputGateState s{};
    TVInputGateInput in = healthy();
    in.steering_valid = false;
    TEST_ASSERT_FALSE(tv_input_gate_compute(in, s).tv_enable);
}

void test_repeated_yaw_value_is_flagged(void) {
    TVInputGateState s{};
    TVInputGateInput in = healthy();
    in.yaw_rate_dps = 10.0f;
    TEST_ASSERT_FALSE(tv_input_gate_compute(in, s).yaw_sample_repeated);  // first
    TEST_ASSERT_TRUE(tv_input_gate_compute(in, s).yaw_sample_repeated);   // re-read
    in.yaw_rate_dps = 10.5f;
    TEST_ASSERT_FALSE(tv_input_gate_compute(in, s).yaw_sample_repeated);  // new
    TEST_ASSERT_TRUE(tv_input_gate_compute(in, s).yaw_sample_repeated);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_all_inputs_healthy_enables_tv);
    RUN_TEST(test_dash_switch_off_blocks_tv);
    RUN_TEST(test_stale_imu_frames_block_tv);
    RUN_TEST(test_frames_without_rate_of_turn_block_tv);
    RUN_TEST(test_invalid_steering_blocks_tv);
    RUN_TEST(test_repeated_yaw_value_is_flagged);
    return UNITY_END();
}
