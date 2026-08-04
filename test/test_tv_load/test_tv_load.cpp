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

void test_positive_ay_shifts_load_to_right(void) {
    // ay>0(좌측 가속) → 우측(바깥)으로 하중 이동 → fz_R > fz_L.
    WheelLoads fz = tv_load_compute(0.0f, 3.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(fz.fz_R > fz.fz_L);
}

void test_negative_ay_shifts_load_to_left(void) {
    // ay<0(우측 가속) → 좌측으로 하중 이동 → fz_L > fz_R.
    WheelLoads fz = tv_load_compute(0.0f, -3.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(fz.fz_L > fz.fz_R);
}

void test_sum_conserved_without_ax(void) {
    // ax=0이면 좌우 합은 횡가속 유무와 상관없이 구동축 정적하중 총합과 같다
    // (횡이동은 안->밖으로 옮길 뿐 총량을 안 바꾼다).
    WheelLoads no_lat = tv_load_compute(0.0f, 0.0f, TV_PARAMS);
    WheelLoads with_lat = tv_load_compute(0.0f, 3.0f, TV_PARAMS);
    float total_no_lat = no_lat.fz_L + no_lat.fz_R;
    float total_with_lat = with_lat.fz_L + with_lat.fz_R;
    TEST_ASSERT_FLOAT_WITHIN(0.1f, total_no_lat, total_with_lat);
}

void test_forward_accel_increases_driven_axle_load(void) {
    // ax>0(전진 가속) → 구동축(후륜) 하중 증가 → 좌우 합이 정적하중보다 커진다.
    WheelLoads no_accel = tv_load_compute(0.0f, 0.0f, TV_PARAMS);
    WheelLoads accel = tv_load_compute(3.0f, 0.0f, TV_PARAMS);
    float total_no_accel = no_accel.fz_L + no_accel.fz_R;
    float total_accel = accel.fz_L + accel.fz_R;
    TEST_ASSERT_TRUE(total_accel > total_no_accel);
}

void test_extreme_ay_clamps_inner_wheel_to_zero(void) {
    // 극단적인 횡가속에서 안쪽 바퀴 하중이 음수 대신 0으로 막힌다.
    WheelLoads fz = tv_load_compute(0.0f, 100.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, fz.fz_L);
    TEST_ASSERT_TRUE(fz.fz_R > 0.0f);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_accel_symmetric_load);
    RUN_TEST(test_positive_ay_shifts_load_to_right);
    RUN_TEST(test_negative_ay_shifts_load_to_left);
    RUN_TEST(test_sum_conserved_without_ax);
    RUN_TEST(test_forward_accel_increases_driven_axle_load);
    RUN_TEST(test_extreme_ay_clamps_inner_wheel_to_zero);
    return UNITY_END();
}
