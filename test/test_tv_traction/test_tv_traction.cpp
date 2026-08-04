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

void test_heavier_wheel_has_more_limit(void) {
    // Fz가 큰 바퀴가 최대전류도 크다 (단조성).
    WheelLoads fz{ 1000.0f, 500.0f };
    MaxTorque m = tv_traction_compute(fz, 0.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(m.max_L > m.max_R);
}

void test_lateral_accel_reduces_limit(void) {
    // 횡가속 |ay|가 커지면 종방향 최대전류가 줄어든다 (마찰원).
    WheelLoads fz{ 800.0f, 800.0f };
    MaxTorque no_lat = tv_traction_compute(fz, 0.0f, TV_PARAMS);
    MaxTorque with_lat = tv_traction_compute(fz, 5.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(with_lat.max_L < no_lat.max_L);
}

void test_higher_mu_gives_more_limit(void) {
    // mu가 클수록 낼 수 있는 최대전류도 커진다.
    WheelLoads fz{ 800.0f, 800.0f };
    TVParams low_mu = TV_PARAMS;
    low_mu.mu = 0.3f;
    TVParams high_mu = TV_PARAMS;
    high_mu.mu = 1.2f;
    MaxTorque lo = tv_traction_compute(fz, 0.0f, low_mu);
    MaxTorque hi = tv_traction_compute(fz, 0.0f, high_mu);
    TEST_ASSERT_TRUE(hi.max_L > lo.max_L);
}

void test_extreme_lateral_clamps_to_zero_not_nan(void) {
    // 횡력이 grip을 넘어서는 극단적 상황에서도 음수sqrt(NaN) 없이 0으로 막힌다.
    WheelLoads fz{ 800.0f, 800.0f };
    MaxTorque m = tv_traction_compute(fz, 1000.0f, TV_PARAMS); // 비현실적으로 큰 ay
    TEST_ASSERT_TRUE(m.max_L >= 0.0f);
    TEST_ASSERT_TRUE(m.max_L == m.max_L); // NaN != NaN 이므로 이 비교가 실패하면 NaN
}

void test_zero_load_gives_zero_limit(void) {
    // 하중이 0(바퀴 들림)인 바퀴는 최대전류도 0.
    WheelLoads fz{ 0.0f, 800.0f };
    MaxTorque m = tv_traction_compute(fz, 0.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, m.max_L);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_equal_load_equal_limit);
    RUN_TEST(test_heavier_wheel_has_more_limit);
    RUN_TEST(test_lateral_accel_reduces_limit);
    RUN_TEST(test_higher_mu_gives_more_limit);
    RUN_TEST(test_extreme_lateral_clamps_to_zero_not_nan);
    RUN_TEST(test_zero_load_gives_zero_limit);
    return UNITY_END();
}
