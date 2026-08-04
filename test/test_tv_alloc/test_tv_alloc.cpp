// Stage 5 — 토크 배분 테스트   담당: ______
// stub·실구현 모두 성립하는 불변식으로 시작. 구현하며 TODO를 채우세요.
#include <unity.h>
#include "modules/tv/allocation.h"

static MaxTorque unlimited() { return { 1.0e6f, 1.0e6f }; }

void test_no_yaw_is_symmetric(void) {
    // Mz=0, 상한 넉넉하면 좌우 균등 & 합=총전류.
    TVAllocOutput o = tv_alloc_compute(20.0f, 0.0f, unlimited(), TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, (float)o.torque_L, (float)o.torque_R);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, (float)o.torque_L + (float)o.torque_R);
}

void test_positive_mz_makes_right_bigger(void) {
    // 양의 Mz(좌회전 보조) → 우측 전류가 좌측보다 커야 한다 (좌표계 규약).
    TVAllocOutput o = tv_alloc_compute(20.0f, 50.0f, unlimited(), TV_PARAMS);
    TEST_ASSERT_TRUE((float)o.torque_R > (float)o.torque_L);
}

void test_zero_mz_zero_torque_is_zero(void) {
    // 총전류 0, Mz 0이면 좌우 다 0.
    TVAllocOutput o = tv_alloc_compute(0.0f, 0.0f, unlimited(), TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, (float)o.torque_L);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, (float)o.torque_R);
}

void test_clamped_to_wheel_limit(void) {
    // 한쪽 상한이 낮으면 그 바퀴 출력은 상한을 넘지 않는다.
    MaxTorque tight{5.0f, 1.0e6f};
    TVAllocOutput o = tv_alloc_compute(20.0f, 0.0f, tight, TV_PARAMS);
    TEST_ASSERT_TRUE((float)o.torque_L <= 5.0f + 0.01f);
}

void test_regen_sign_preserved_when_unsaturated(void) {
    // 회생(총전류 음수)이고 포화 없는 상황이면 좌우 다 음수여야 한다.
    TVAllocOutput o = tv_alloc_compute(-20.0f, 0.0f, unlimited(), TV_PARAMS);
    TEST_ASSERT_TRUE((float)o.torque_L < 0.0f);
    TEST_ASSERT_TRUE((float)o.torque_R < 0.0f);
}

// TODO(담당자): 아직 미정 — "한쪽이 상한에 걸릴 때 총전류 우선이냐 yaw 우선이냐"
// 정책을 팀에서 정하고 나면, 그 정책을 검증하는 테스트를 여기 추가할 것.

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_yaw_is_symmetric);
    RUN_TEST(test_positive_mz_makes_right_bigger);
    RUN_TEST(test_zero_mz_zero_torque_is_zero);
    RUN_TEST(test_clamped_to_wheel_limit);
    RUN_TEST(test_regen_sign_preserved_when_unsaturated);
    return UNITY_END();
}
