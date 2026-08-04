// Stage 4 — 트랙션 한계 테스트   담당: ______
// stub·실구현 모두 성립하는 불변식으로 시작. 구현하며 TODO를 채우세요.
#include <unity.h>
#include "modules/tv/traction.h"

void test_equal_load_equal_limit(void) {
    // 좌우 하중이 같고 횡가속 0이면 좌우 최대토크도 같다.
    WheelLoads fz{ 800.0f, 800.0f };
    MaxTorque m = tv_traction_compute(fz, 0.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, m.max_L, m.max_R);
    TEST_ASSERT_TRUE(m.max_L > 0.0f);
}

// Fz가 큰 바퀴가 최대토크도 크다 (단조성)
void test_monotonic_in_load(void) {
    MaxTorque m = tv_traction_compute(WheelLoads{ 400.0f, 900.0f }, 0.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(m.max_R > m.max_L);
}

// |ay|가 커지면 종방향 최대토크가 줄어든다 (마찰원)
void test_friction_circle_reduces_with_lateral(void) {
    WheelLoads fz{ 800.0f, 800.0f };
    float no_lat  = tv_traction_compute(fz, 0.0f, TV_PARAMS).max_L;
    float w_lat   = tv_traction_compute(fz, 3.0f, TV_PARAMS).max_L;
    TEST_ASSERT_TRUE(w_lat < no_lat);
}

// μ에 비례해 최대토크가 커진다
void test_proportional_to_mu(void) {
    WheelLoads fz{ 800.0f, 800.0f };
    TVParams lo = TV_PARAMS; lo.mu = 0.5f;
    TVParams hi = TV_PARAMS; hi.mu = 1.0f;
    float t_lo = tv_traction_compute(fz, 0.0f, lo).max_L;
    float t_hi = tv_traction_compute(fz, 0.0f, hi).max_L;
    TEST_ASSERT_TRUE(t_hi > t_lo);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 2.0f * t_lo, t_hi);   // μ 2배 → 종여유(=μFz) 2배
}

// 횡한계 초과에서 sqrt 음수 없이 0으로 막힌다
void test_lateral_saturation_clamps_to_zero(void) {
    // 작은 하중 + 큰 횡가속 → Fy > μ·Fz → 종여유 0
    WheelLoads fz{ 50.0f, 50.0f };
    MaxTorque m = tv_traction_compute(fz, 30.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, m.max_L);
    TEST_ASSERT_TRUE(m.max_L >= 0.0f);   // 음수·NaN 아님
}

// Fz=0이면 상한도 0
void test_zero_load_zero_limit(void) {
    MaxTorque m = tv_traction_compute(WheelLoads{ 0.0f, 0.0f }, 0.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, m.max_L);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, m.max_R);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_equal_load_equal_limit);
    RUN_TEST(test_monotonic_in_load);
    RUN_TEST(test_friction_circle_reduces_with_lateral);
    RUN_TEST(test_proportional_to_mu);
    RUN_TEST(test_lateral_saturation_clamps_to_zero);
    RUN_TEST(test_zero_load_zero_limit);
    return UNITY_END();
}
