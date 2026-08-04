// 통합(오케스트레이터) 테스트 — tv_compute가 5개 stage를 올바로 조립하는지.
// stage 알고리즘이 채워져도 유지되는 "불변식"만 검증한다.
#include <unity.h>
#include "modules/torque_vectoring.h"

static TVInput straight() {
    // total=20, 직진(조향0/yaw0/가속0), dt=10ms
    return TVInput{ 20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f };
}

// total=20, 좌조향+측정yaw+횡가속, 차속 speed
static TVInput turning(float speed) {
    return TVInput{ 20.0f, 5.0f, 0.5f, speed, 0.0f, 0.1f, 0.01f };
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
void test_intermediates_are_populated(void) {
    // 중간신호가 TVOutput에 실려 나오는지 (관측 경로 확인)
    TVYawState s{};
    TVOutput o = tv_compute(straight(), s);
    TEST_ASSERT_TRUE(o.fz_L > 0.0f && o.fz_R > 0.0f);   // 정적하중은 양수
    TEST_ASSERT_TRUE(o.max_torque_L > 0.0f);            // 트랙션 상한 존재
}

// 저속: 조향 줘도 목표 yaw 0 + 좌우 대칭 (TV 전체 차단 §4)
void test_low_speed_disables_tv(void) {
    TVYawState s{};
    TVOutput lo = tv_compute(turning(TV_PARAMS.tv_min_speed_mps * 0.5f), s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, lo.desired_yaw_rate);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, (float)lo.torque_L, (float)lo.torque_R);
}

// 저속 차단 구간에선 yaw 적분이 리셋된다 (재활성 와인드업 방지)
void test_low_speed_resets_integrator(void) {
    TVYawState s{}; s.integral = 3.0f;
    tv_compute(turning(TV_PARAMS.tv_min_speed_mps * 0.5f), s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, s.integral);
}

// 고속에선 목표 yaw가 살아있다 (컷오프가 속도 게이트임을 확인)
void test_high_speed_enables_reference(void) {
    TVYawState s{};
    TVOutput hi = tv_compute(turning(10.0f), s);
    TEST_ASSERT_TRUE(hi.desired_yaw_rate > 0.0f);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_straight_is_symmetric);
    RUN_TEST(test_split_sums_to_demand);
    RUN_TEST(test_intermediates_are_populated);
    RUN_TEST(test_low_speed_disables_tv);
    RUN_TEST(test_low_speed_resets_integrator);
    RUN_TEST(test_high_speed_enables_reference);
    return UNITY_END();
}
