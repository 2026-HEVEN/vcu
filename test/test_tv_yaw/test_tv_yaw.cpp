// Stage 2 — yaw 제어기 테스트   담당: ______
// stub·실구현 모두 성립하는 불변식으로 시작. 구현하며 TODO를 채우세요.
#include <unity.h>
#include "modules/tv/yaw_control.h"

// 게인 주입한 로컬 TVParams (TV_PARAMS는 kp/ki/kd=0이라 그대로 쓰면 무의미)
static TVParams tuned(float kp, float ki, float kd, float mzmax = 200.0f) {
    TVParams g{};
    g.kp = kp; g.ki = ki; g.kd = kd; g.yaw_moment_max = mzmax;
    return g;
}

void test_zero_error_zero_moment(void) {
    // 목표 == 실측 (오차 0), 갓 초기화한 상태면 Mz는 0.
    TVYawState s{};
    float mz = tv_yaw_compute(10.0f, 10.0f, 0.01f, TV_PARAMS, s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, mz);
}

// 비례항: Mz = Kp·오차, 부호도 오차 따라감
void test_proportional_sign_and_magnitude(void) {
    TVParams g = tuned(2.0f, 0.0f, 0.0f);
    TVYawState s1{}, s2{};
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.0f,  tv_yaw_compute(10.0f, 4.0f, 0.01f, g, s1));  // +6→+12
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -12.0f, tv_yaw_compute(4.0f, 10.0f, 0.01f, g, s2));  // -6→-12
}

// 양(+) 오차가 지속되면 적분이 쌓여 Mz가 커진다
void test_integral_accumulates(void) {
    TVParams g = tuned(0.0f, 1.0f, 0.0f);   // 순수 적분
    TVYawState s{};
    float m1 = tv_yaw_compute(5.0f, 0.0f, 0.1f, g, s);   // I=0.5
    float m2 = tv_yaw_compute(5.0f, 0.0f, 0.1f, g, s);   // I=1.0
    float m3 = tv_yaw_compute(5.0f, 0.0f, 0.1f, g, s);   // I=1.5
    TEST_ASSERT_TRUE(m2 > m1 && m3 > m2);
}

// 출력이 yaw_moment_max로 clamp
void test_output_clamped_to_moment_max(void) {
    TVParams g = tuned(100.0f, 0.0f, 0.0f, 200.0f);
    TVYawState s{};
    float mz = tv_yaw_compute(10.0f, 0.0f, 0.01f, g, s);  // 100*10=1000 → 200
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, mz);
}

// anti-windup: 포화 중이면 적분이 계속 쌓이지 않는다 (없으면 폭주)
void test_anti_windup_stops_integrating(void) {
    TVParams g = tuned(0.0f, 1.0f, 0.0f, 10.0f);  // 순수 적분, 상한 10
    TVYawState s{};
    tv_yaw_compute(6.0f, 0.0f, 1.0f, g, s);       // I=6 (u=6, 미포화)
    tv_yaw_compute(6.0f, 0.0f, 1.0f, g, s);       // u=12>10 포화 → 적분 동결
    float I_frozen = s.integral;
    tv_yaw_compute(6.0f, 0.0f, 1.0f, g, s);
    tv_yaw_compute(6.0f, 0.0f, 1.0f, g, s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, I_frozen, s.integral);  // 안 늘어남
    TEST_ASSERT_TRUE(s.integral < 12.0f);   // anti-windup 없었으면 24,30...로 폭주
}

// 상태 주입 → 결정론적 재현
void test_state_injection_is_deterministic(void) {
    TVParams g = tuned(1.0f, 2.0f, 0.5f);
    TVYawState a{}; a.integral = 3.0f; a.prev_error = 1.0f;
    TVYawState b = a;
    float ma = tv_yaw_compute(5.0f, 2.0f, 0.1f, g, a);
    float mb = tv_yaw_compute(5.0f, 2.0f, 0.1f, g, b);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, ma, mb);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, a.integral, b.integral);
}

// dt=0 방어: 0으로 나눔·적분 누적 없음
void test_dt_zero_is_safe(void) {
    TVParams g = tuned(1.0f, 1.0f, 1.0f);
    TVYawState s{};
    float mz = tv_yaw_compute(5.0f, 0.0f, 0.0f, g, s);
    TEST_ASSERT_TRUE(mz == mz);                            // NaN 아님
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, s.integral);   // dt=0 → 적분 누적 안 함
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.0f, mz);            // kp*error만 남음
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_error_zero_moment);
    RUN_TEST(test_proportional_sign_and_magnitude);
    RUN_TEST(test_integral_accumulates);
    RUN_TEST(test_output_clamped_to_moment_max);
    RUN_TEST(test_anti_windup_stops_integrating);
    RUN_TEST(test_state_injection_is_deterministic);
    RUN_TEST(test_dt_zero_is_safe);
    return UNITY_END();
}
