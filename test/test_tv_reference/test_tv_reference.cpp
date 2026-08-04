// Stage 1 — 레퍼런스 모델 테스트   담당: ______
// 아래는 stub·실구현 모두에서 성립하는 불변식. 구현하며 TODO 테스트를 채우세요.
#include <unity.h>
#include "modules/tv/reference.h"

void test_no_steer_no_target(void) {
    // 직진(조향 0)이면 목표 yaw rate는 0.
    float dy = tv_reference_compute(Unit(0.0f), 15.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, dy);
}

void test_below_min_speed_is_zero(void) {
    // 저속 컷오프 미만이면 조향을 아무리 줘도 목표 yaw는 0.
    float dy = tv_reference_compute(Unit(1.0f), 0.2f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, dy);
}

void test_steer_sign_matches_yaw_convention(void) {
    // 좌회전 조향(+)이면 목표 yaw도 +(좌회전), 우회전(-)이면 -.
    float left = tv_reference_compute(Unit(0.3f), 10.0f, TV_PARAMS);
    float right = tv_reference_compute(Unit(-0.3f), 10.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(left > 0.0f);
    TEST_ASSERT_TRUE(right < 0.0f);
}

void test_larger_steer_gives_larger_target(void) {
    // 조향이 커질수록(같은 부호) |목표 yaw|도 커진다 (단조성).
    float small = tv_reference_compute(Unit(0.2f), 10.0f, TV_PARAMS);
    float large = tv_reference_compute(Unit(0.6f), 10.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(large > small);
}

void test_symmetry_left_right(void) {
    // 좌/우 조향은 크기가 같으면 목표 yaw도 부호만 다르고 크기는 같다.
    float left = tv_reference_compute(Unit(0.4f), 12.0f, TV_PARAMS);
    float right = tv_reference_compute(Unit(-0.4f), 12.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -left, right);
}

void test_output_never_exceeds_desired_yaw_max(void) {
    // 풀 스티어 + 저속(분모 작음)이라도 상한을 넘지 않는다.
    float dy = tv_reference_compute(Unit(1.0f), 1.5f, TV_PARAMS);
    TEST_ASSERT_TRUE(dy <= TV_PARAMS.desired_yaw_max + 0.001f);
    TEST_ASSERT_TRUE(dy >= -TV_PARAMS.desired_yaw_max - 0.001f);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_steer_no_target);
    RUN_TEST(test_below_min_speed_is_zero);
    RUN_TEST(test_steer_sign_matches_yaw_convention);
    RUN_TEST(test_larger_steer_gives_larger_target);
    RUN_TEST(test_symmetry_left_right);
    RUN_TEST(test_output_never_exceeds_desired_yaw_max);
    return UNITY_END();
}
