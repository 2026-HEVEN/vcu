// Stage 2 — yaw 제어기 테스트   담당: ______
// stub·실구현 모두 성립하는 불변식으로 시작. 구현하며 TODO를 채우세요.
#include <unity.h>
#include "modules/tv/yaw_control.h"

void test_zero_error_zero_moment(void) {
    // 목표 == 실측 (오차 0), 갓 초기화한 상태면 Mz는 0.
    TVYawState s{};
    float mz = tv_yaw_compute(10.0f, 10.0f, 0.01f, TV_PARAMS, s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, mz);
}

void test_positive_error_gives_positive_moment(void) {
    // P항만 켜둔 파라미터: 목표가 실측보다 크면(양의 오차) Mz도 양수.
    TVParams p = TV_PARAMS;
    p.kp = 2.0f;
    p.yaw_moment_max = 100.0f;
    TVYawState s{};
    float mz = tv_yaw_compute(20.0f, 5.0f, 0.01f, p, s); // error = 15
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, mz); // kp*error = 2*15
}

void test_integral_accumulates_over_ticks(void) {
    // I항만 켜고, 같은 양의 오차를 여러 tick 넣으면 적분이 쌓여 Mz가 점점 커진다.
    TVParams p = TV_PARAMS;
    p.ki = 1.0f;
    p.yaw_moment_max = 1000.0f; // 이 테스트에서는 포화 안 걸리게 크게
    TVYawState s{};
    float mz1 = tv_yaw_compute(10.0f, 0.0f, 0.1f, p, s); // error=10, integral=1.0
    float mz2 = tv_yaw_compute(10.0f, 0.0f, 0.1f, p, s); // integral=2.0
    TEST_ASSERT_TRUE(mz2 > mz1);
}

void test_output_clamped_to_yaw_moment_max(void) {
    // 큰 오차 + 큰 게인이면 상한을 넘지 않는다.
    TVParams p = TV_PARAMS;
    p.kp = 1000.0f;
    p.yaw_moment_max = 50.0f;
    TVYawState s{};
    float mz = tv_yaw_compute(100.0f, 0.0f, 0.01f, p, s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, mz);
}

void test_anti_windup_stops_integral_when_saturated(void) {
    // 이미 출력이 포화된 방향으로 계속 미는 오차는 적분에 반영되지 않는다.
    // → 포화 이후 여러 tick을 더 돌려도 s.integral이 더 이상 커지지 않아야 한다.
    TVParams p = TV_PARAMS;
    p.ki = 10.0f;
    p.yaw_moment_max = 5.0f;
    TVYawState s{};
    for (int i = 0; i < 5; ++i) {
        tv_yaw_compute(100.0f, 0.0f, 0.1f, p, s); // 항상 큰 양의 오차, 계속 포화
    }
    float integral_after_saturation = s.integral;
    tv_yaw_compute(100.0f, 0.0f, 0.1f, p, s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, integral_after_saturation, s.integral);
}

void test_non_positive_dt_returns_zero(void) {
    // dt<=0 이면 0으로 나누지 않고 안전하게 0을 반환한다.
    TVParams p = TV_PARAMS;
    p.kp = 5.0f;
    TVYawState s{};
    float mz = tv_yaw_compute(20.0f, 0.0f, 0.0f, p, s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mz);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_error_zero_moment);
    RUN_TEST(test_positive_error_gives_positive_moment);
    RUN_TEST(test_integral_accumulates_over_ticks);
    RUN_TEST(test_output_clamped_to_yaw_moment_max);
    RUN_TEST(test_anti_windup_stops_integral_when_saturated);
    RUN_TEST(test_non_positive_dt_returns_zero);
    return UNITY_END();
}
