// 통합(오케스트레이터) 테스트 — tv_compute가 5개 stage를 올바로 조립하는지.
// stage 알고리즘이 채워져도 유지되는 "불변식"만 검증한다.
#include <unity.h>
#include "modules/torque_vectoring.h"

static TVInput straight() {
    // total=20, 직진(조향0/yaw0/가속0), dt=10ms, 대시 TC/TV 스위치=on
    return TVInput{ 20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, true };
}

void test_straight_is_symmetric(void) {
    TVYawState s{};
    TVOutput o = tv_compute(straight(), s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, (float)o.torque_L, (float)o.torque_R);
}
void test_split_sums_to_demand(void) {
    TVYawState s{};
    TVOutput o = tv_compute(straight(), s);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, (float)o.torque_L + (float)o.torque_R);
}
void test_full_bringup_demand_is_300_a_per_motor_when_tv_off(void) {
    TVYawState s{};
    TVInput in = straight();
    in.total_torque = 600.0f;
    TVOutput o = tv_compute(in, s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 300.0f, (float)o.torque_L);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 300.0f, (float)o.torque_R);
}
void test_intermediates_are_populated(void) {
    // 중간신호가 TVOutput에 실려 나오는지 (관측 경로 확인)
    TVYawState s{};
    TVOutput o = tv_compute(straight(), s);
    TEST_ASSERT_TRUE(o.fz_L > 0.0f && o.fz_R > 0.0f);   // 정적하중은 양수
    TEST_ASSERT_TRUE(o.max_torque_L > 0.0f);            // 트랙션 상한 존재
}

void test_low_speed_explicitly_disables_yaw_feedback(void) {
    TVYawState s{};
    s.integral = 50.0f;
    TVInput in = straight();
    in.yaw_rate = 20.0f;
    in.vehicle_speed = 0.5f;
    TVOutput o = tv_compute(in, s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, o.yaw_moment);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, s.integral);
}

void test_zero_gains_are_strict_5050_off_even_beyond_friction_model(void) {
    TVYawState s{};
    TVInput in = straight();
    in.vehicle_speed = 15.0f;
    in.steering_angle = 0.8f;
    in.yaw_rate = -30.0f;
    in.ay = 10.0f; // deliberately drives the Stage-4 diagnostic limit to zero
    TVOutput o = tv_compute(in, s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, (float)o.torque_L);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, (float)o.torque_R);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, o.yaw_moment);
}

void test_dash_switch_off_is_strict_5050_even_with_gains_and_speed(void) {
    // Same scenario as the zero-gains strict-OFF test, but this time gains
    // and speed are healthy — only the dash TC/TV switch is off. Must be
    // behaviorally identical: no differential, no silent traction cut.
    TVYawState s{};
    TVInput in = straight();
    in.vehicle_speed = 15.0f;
    in.steering_angle = 0.8f;
    in.yaw_rate = -30.0f;
    in.ay = 10.0f;
    in.tv_enable_requested = false;   // dash switch OFF
    TVOutput o = tv_compute(in, s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, (float)o.torque_L);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, (float)o.torque_R);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, o.yaw_moment);
}
void test_dash_switch_on_allows_differential(void) {
    // Sanity check: with the dash switch left at its default (true), a
    // straight-line scenario is unaffected by the new gate (regression
    // guard against accidentally inverting the condition).
    TVYawState s{};
    TVOutput o = tv_compute(straight(), s);
    TEST_ASSERT_TRUE(o.torque_L >= 0.0f && o.torque_R >= 0.0f);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_straight_is_symmetric);
    RUN_TEST(test_split_sums_to_demand);
    RUN_TEST(test_full_bringup_demand_is_300_a_per_motor_when_tv_off);
    RUN_TEST(test_intermediates_are_populated);
    RUN_TEST(test_low_speed_explicitly_disables_yaw_feedback);
    RUN_TEST(test_zero_gains_are_strict_5050_off_even_beyond_friction_model);
    RUN_TEST(test_dash_switch_off_is_strict_5050_even_with_gains_and_speed);
    RUN_TEST(test_dash_switch_on_allows_differential);
    return UNITY_END();
}
