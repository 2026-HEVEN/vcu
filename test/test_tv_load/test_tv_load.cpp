// Stage 3 — 하중 추정 테스트   담당: ______
// stub·실구현 모두 성립하는 불변식으로 시작. 구현하며 TODO를 채우세요.
#include <unity.h>
#include "modules/tv/load.h"

void test_no_accel_symmetric_load(void) {
    // 가속 없음(ax=ay=0)이면 좌우 하중은 대칭.
    WheelLoads fz = tv_load_compute(0.0f, 0.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, fz.fz_L, fz.fz_R);
    TEST_ASSERT_TRUE(fz.fz_L > 0.0f);   // 정적 하중은 양수
}

// 정지 시 좌우 합 = 구동축 정적하중 (m_axle·g)
void test_static_load_matches_axle_weight(void) {
    WheelLoads fz = tv_load_compute(0.0f, 0.0f, TV_PARAMS);
    float axle_w = TV_PARAMS.mass_kg * TV_PARAMS.weight_dist_r * 9.81f;
    TEST_ASSERT_FLOAT_WITHIN(1.0f, axle_w, fz.fz_L + fz.fz_R);
}

// ay>0(좌회전) → 우측(바깥) 하중이 좌측(안쪽)보다 크다
void test_lateral_transfer_goes_outside(void) {
    WheelLoads fz = tv_load_compute(0.0f, 3.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(fz.fz_R > fz.fz_L);
}

// 횡이동은 축 총하중을 보존 (좌우 합 불변)
void test_lateral_transfer_conserves_sum(void) {
    WheelLoads z0 = tv_load_compute(0.0f, 0.0f, TV_PARAMS);
    WheelLoads z1 = tv_load_compute(0.0f, 4.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, z0.fz_L + z0.fz_R, z1.fz_L + z1.fz_R);
}

// ax>0(가속) → 구동축(후) 총하중 증가
void test_longitudinal_transfer_sign(void) {
    WheelLoads z0 = tv_load_compute(0.0f, 0.0f, TV_PARAMS);
    WheelLoads zp = tv_load_compute(5.0f, 0.0f, TV_PARAMS);   // 가속
    TEST_ASSERT_TRUE(zp.fz_L + zp.fz_R > z0.fz_L + z0.fz_R);
}

// 극단적 ay에서 안쪽 Fz가 0으로 clamp되고 음수가 안 나온다
void test_inner_wheel_clamped_at_zero(void) {
    WheelLoads fz = tv_load_compute(0.0f, 50.0f, TV_PARAMS);   // 비현실적 큰 횡가속
    TEST_ASSERT_TRUE(fz.fz_L >= 0.0f);   // 음수 없음
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, fz.fz_L);   // 안쪽은 0으로 막힘
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_accel_symmetric_load);
    RUN_TEST(test_static_load_matches_axle_weight);
    RUN_TEST(test_lateral_transfer_goes_outside);
    RUN_TEST(test_lateral_transfer_conserves_sum);
    RUN_TEST(test_longitudinal_transfer_sign);
    RUN_TEST(test_inner_wheel_clamped_at_zero);
    return UNITY_END();
}
