#include <unity.h>
#include "modules/tv/load.h"

namespace { constexpr float G = 9.80665f; }

// 차원 타입 도입 후에도 기존 검증 의도를 그대로 두기 위한 float 어댑터.
// stage 시그니처는 GForce/Newton을 요구하고, 테스트 본문은 숫자로 읽는 게 낫다.
struct FzF { float fz_L, fz_R; };
static FzF load_f(float ax_g, float ay_g, const TVParams &p) {
    const WheelLoads w = tv_load_compute(GForce{ax_g}, GForce{ay_g}, p);
    return { (float)w.fz_L, (float)w.fz_R };
}

void test_static_load_is_symmetric() {
    const FzF fz = load_f(0.0f, 0.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, fz.fz_L, fz.fz_R);
    TEST_ASSERT_TRUE(fz.fz_L > 0.0f);
}

void test_positive_lateral_g_loads_right_wheel() {
    const FzF fz = load_f(0.0f, 0.5f, TV_PARAMS);
    TEST_ASSERT_TRUE(fz.fz_R > fz.fz_L);
}

void test_forward_acceleration_increases_rear_load() {
    const FzF steady = load_f(0.0f, 0.0f, TV_PARAMS);
    const FzF accel = load_f(0.5f, 0.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(accel.fz_L + accel.fz_R > steady.fz_L + steady.fz_R);
}

void test_extreme_input_never_returns_negative_load() {
    const FzF fz = load_f(-10.0f, 10.0f, TV_PARAMS);
    TEST_ASSERT_TRUE(fz.fz_L >= 0.0f && fz.fz_R >= 0.0f);
}

void test_lateral_transfer_conserves_rear_axle_sum() {
    const FzF steady = load_f(0.0f, 0.0f, TV_PARAMS);
    const FzF corner = load_f(0.0f, 0.7f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, steady.fz_L + steady.fz_R,
                            corner.fz_L + corner.fz_R);
}

void test_lltd_is_independent_from_static_distribution() {
    TVParams low = TV_PARAMS, high = TV_PARAMS;
    low.lltd_r = 0.2f; high.lltd_r = 0.8f;
    const FzF a = load_f(0.0f, 0.5f, low);
    const FzF b = load_f(0.0f, 0.5f, high);
    TEST_ASSERT_TRUE((b.fz_R - b.fz_L) > (a.fz_R - a.fz_L));
}

// ── 크기 검증 ────────────────────────────────────────────────────────────
// 위 테스트들은 전부 "크다/작다"만 본다. 그래서 횡하중 이동 계수가 2배 틀려도
// 아무도 잡지 못했다. 롤 모멘트 평형으로 절대값을 고정한다.

void test_lateral_transfer_satisfies_roll_moment_balance() {
    // 후축이 받는 롤 모멘트 몫  =  lltd_r · m · ay · h
    // 이를 버티는 건 좌우 수직하중 차이:  (F_R − F_L) · (t/2)
    //   →  F_R − F_L = 2 · lltd_r · m · ay · h / t
    const TVParams &p = TV_PARAMS;
    const float ay_g = 0.6f;
    const FzF fz = load_f(0.0f, ay_g, p);

    const float roll_moment_rear = p.lltd_r * p.mass_kg * (ay_g * G) * p.cg_height_m;
    const float reacting_moment = (fz.fz_R - fz.fz_L) * (p.track_m * 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, roll_moment_rear, reacting_moment);
}

void test_each_wheel_deviates_by_the_axle_transfer() {
    // 바퀴당 편차 = lltd_r · m · ay · h / t  (축 합계의 절반에서 ±)
    const TVParams &p = TV_PARAMS;
    const float ay_g = 0.6f;
    const FzF fz = load_f(0.0f, ay_g, p);

    const float half_axle = 0.5f * p.mass_kg * G * p.weight_dist_r;
    const float expected = p.lltd_r * p.mass_kg * (ay_g * G) * p.cg_height_m / p.track_m;
    TEST_ASSERT_FLOAT_WITHIN(0.5f, half_axle - expected, fz.fz_L);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, half_axle + expected, fz.fz_R);
}

void test_inside_wheel_lifts_before_the_car_would_tip_over() {
    // 물리적 정합성 검사. lltd_r = 1.0 이면 횡하중 이동이 전부 후축에 실리므로
    // 안쪽 뒷바퀴는 비교적 쉽게 뜬다. 차량 전체 정적 전복 임계 t/(2h) 보다
    // 늦게 뜬다면 식이 틀린 것이다 — 한 축의 바퀴가 차 전체가 뒤집힌 뒤에
    // 뜰 수는 없다.
    TVParams p = TV_PARAMS;
    p.lltd_r = 1.0f;
    const float static_rollover_g = p.track_m / (2.0f * p.cg_height_m);  // 1.95 g
    const float ay_g = 1.5f;   // 전복 임계보다 낮게 잡는다
    TEST_ASSERT_TRUE(ay_g < static_rollover_g);

    const FzF fz = load_f(0.0f, ay_g, p);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, fz.fz_L);   // 안쪽은 이미 떠 있어야 한다
}

void setUp() {}
void tearDown() {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_static_load_is_symmetric);
    RUN_TEST(test_positive_lateral_g_loads_right_wheel);
    RUN_TEST(test_forward_acceleration_increases_rear_load);
    RUN_TEST(test_extreme_input_never_returns_negative_load);
    RUN_TEST(test_lateral_transfer_conserves_rear_axle_sum);
    RUN_TEST(test_lltd_is_independent_from_static_distribution);
    RUN_TEST(test_lateral_transfer_satisfies_roll_moment_balance);
    RUN_TEST(test_each_wheel_deviates_by_the_axle_transfer);
    RUN_TEST(test_inside_wheel_lifts_before_the_car_would_tip_over);
    return UNITY_END();
}
