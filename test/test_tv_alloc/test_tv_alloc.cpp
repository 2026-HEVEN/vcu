// Stage 5 — 토크 배분 테스트   담당: ______
// stub·실구현 모두 성립하는 불변식으로 시작. 구현하며 TODO를 채우세요.
#include <unity.h>
#include "modules/tv/allocation.h"

static MaxTorque unlimited() { return { 1.0e6f, 1.0e6f }; }

// Mz[N·m] → 편도 전류차 ΔI[A] (구현과 동일 환산 §8). 좌우차 = 2·ΔI.
static float alloc_diff(float mz) {
    return mz * TV_PARAMS.tire_radius_m /
           (TV_PARAMS.track_m * TV_PARAMS.kt_nm_per_a * TV_PARAMS.gear_ratio);
}

void test_no_yaw_is_symmetric(void) {
    // Mz=0, 상한 넉넉하면 좌우 균등 & 합=총전류.
    TVAllocOutput o = tv_alloc_compute(20.0f, 0.0f, unlimited(), TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, (float)o.torque_L, (float)o.torque_R);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, (float)o.torque_L + (float)o.torque_R);
}

// Mz>0 → 우측 바퀴 전류가 크고, 좌우차 = 2·ΔI(Mz) (§2.1, §8)
void test_positive_yaw_favours_right_wheel(void) {
    TVAllocOutput o = tv_alloc_compute(20.0f, 8.0f, unlimited(), TV_PARAMS);
    TEST_ASSERT_TRUE((float)o.torque_R > (float)o.torque_L);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f * alloc_diff(8.0f),
                             (float)o.torque_R - (float)o.torque_L);
}

// Mz 부호 반전은 좌우 대칭 (미러)
void test_negative_yaw_is_mirrored(void) {
    TVAllocOutput pos = tv_alloc_compute(20.0f,  8.0f, unlimited(), TV_PARAMS);
    TVAllocOutput neg = tv_alloc_compute(20.0f, -8.0f, unlimited(), TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, (float)pos.torque_R, (float)neg.torque_L);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, (float)pos.torque_L, (float)neg.torque_R);
}

// 바퀴별 상한을 넘지 않는다
void test_traction_limit_is_respected(void) {
    MaxTorque lim{ 30.0f, 30.0f };
    TVAllocOutput o = tv_alloc_compute(50.0f, 20.0f, lim, TV_PARAMS);
    TEST_ASSERT_TRUE((float)o.torque_L <= 30.01f && (float)o.torque_L >= -30.01f);
    TEST_ASSERT_TRUE((float)o.torque_R <= 30.01f && (float)o.torque_R >= -30.01f);
}

// 포화 시 yaw(차등)를 지키고 총량을 희생 (yaw 우선 정책)
void test_saturation_policy_keeps_yaw(void) {
    MaxTorque lim{ 30.0f, 30.0f };
    TVAllocOutput o = tv_alloc_compute(50.0f, 20.0f, lim, TV_PARAMS);   // 총전류 50 요구
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 2.0f * alloc_diff(20.0f),
                             (float)o.torque_R - (float)o.torque_L);    // yaw(=2·ΔI) 유지
    TEST_ASSERT_TRUE((float)o.torque_L + (float)o.torque_R < 50.0f);    // 총량 희생
}

// 실현 불가능한 차등은 실현 가능한 값까지만 clamp (2·d_max)
void test_diff_clamped_when_infeasible(void) {
    MaxTorque lim{ 15.0f, 15.0f };
    TVAllocOutput o = tv_alloc_compute(0.0f, 100.0f, lim, TV_PARAMS);   // 큰 Mz → ΔI가 상한 초과
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 30.0f, (float)o.torque_R - (float)o.torque_L);  // 2·15=30으로 clamp
    TEST_ASSERT_TRUE((float)o.torque_R <= 15.01f);
}

// 회생(총전류 음수) 구간에서 부호 보존 (구동으로 안 뒤집힘)
void test_regen_sign_preserved(void) {
    TVAllocOutput o = tv_alloc_compute(-20.0f, 8.0f, unlimited(), TV_PARAMS);
    TEST_ASSERT_TRUE((float)o.torque_L < 0.0f && (float)o.torque_R < 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -20.0f, (float)o.torque_L + (float)o.torque_R);
}

// 그립 0이면 출력 0
void test_zero_limit_means_zero_output(void) {
    TVAllocOutput o = tv_alloc_compute(20.0f, 8.0f, MaxTorque{ 0.0f, 0.0f }, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, (float)o.torque_L);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, (float)o.torque_R);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_yaw_is_symmetric);
    RUN_TEST(test_positive_yaw_favours_right_wheel);
    RUN_TEST(test_negative_yaw_is_mirrored);
    RUN_TEST(test_traction_limit_is_respected);
    RUN_TEST(test_saturation_policy_keeps_yaw);
    RUN_TEST(test_diff_clamped_when_infeasible);
    RUN_TEST(test_regen_sign_preserved);
    RUN_TEST(test_zero_limit_means_zero_output);
    return UNITY_END();
}
