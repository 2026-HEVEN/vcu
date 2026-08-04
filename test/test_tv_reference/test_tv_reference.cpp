// Stage 1 — 레퍼런스 모델 테스트   담당: ______
// 아래는 stub·실구현 모두에서 성립하는 불변식. 구현하며 TODO 테스트를 채우세요.
#include <unity.h>
#include "modules/tv/reference.h"

void test_no_steer_no_target(void) {
    // 직진(조향 0)이면 목표 yaw rate는 0.
    float dy = tv_reference_compute(Unit(0.0f), 15.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, dy);
}

// 조향이 커지면 |목표 yaw|도 커진다 (단조성). 클램프 안 걸리는 저속 구간에서.
void test_monotonic_in_steering(void) {
    float small = tv_reference_compute(Unit(0.2f), 5.0f, TV_PARAMS);
    float big   = tv_reference_compute(Unit(0.4f), 5.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(big > small && small > 0.0f);   // 둘 다 상한 아래라 비례
}

// 좌/우 조향의 부호가 IMU yaw 규약과 일치 (좌+ / 우−, 크기 대칭)
void test_sign_matches_imu_convention(void) {
    float left  = tv_reference_compute(Unit(+0.5f), 10.0f, TV_PARAMS);
    float right = tv_reference_compute(Unit(-0.5f), 10.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(left > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -left, right);
}

// 어떤 입력에서도 desired_yaw_max를 넘지 않는다 (clamp)
void test_clamped_to_desired_yaw_max(void) {
    // 풀 조향 + 중속 → 클수록 상한에 걸림
    float dy = tv_reference_compute(Unit(1.0f), 20.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(dy <= TV_PARAMS.desired_yaw_max + 0.01f);
}

// 고속에서 마찰 한계가 desired_yaw_max보다 먼저 걸린다 (r_max=μg/V)
void test_friction_limit_caps_high_speed(void) {
    // V가 크면 μg/V(deg/s)가 desired_yaw_max(60)보다 작아짐. μ=1,g=9.81 → 60deg/s=1.047rad/s
    // r_fric=9.81/V*57.3 < 60  ⇔  V > 9.36 m/s
    float dy = tv_reference_compute(Unit(1.0f), 30.0f, TV_PARAMS);
    float r_fric = (TV_PARAMS.mu * 9.81f / 30.0f) * 57.29578f;   // ≈ 18.7 deg/s
    TEST_ASSERT_TRUE(dy <= r_fric + 0.01f);
    TEST_ASSERT_TRUE(dy < TV_PARAMS.desired_yaw_max);   // 마찰 한계가 더 낮게 걸림
}

// 저속(임계 미만)에선 조향을 줘도 목표 yaw = 0 (저속 컷오프 §4)
void test_below_min_speed_is_zero(void) {
    float dy = tv_reference_compute(Unit(0.5f), TV_PARAMS.tv_min_speed_mps * 0.5f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, dy);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_steer_no_target);
    RUN_TEST(test_monotonic_in_steering);
    RUN_TEST(test_sign_matches_imu_convention);
    RUN_TEST(test_clamped_to_desired_yaw_max);
    RUN_TEST(test_friction_limit_caps_high_speed);
    RUN_TEST(test_below_min_speed_is_zero);
    return UNITY_END();
}
