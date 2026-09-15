// TV 게이트 단일 판정자 테스트.
// 이 파일이 존재하는 이유: 게이트 조건이 torque_vectoring.cpp와 can_bus.cpp에
// 따로 구현돼 있어서 드리프트 가능했다. 판정은 tv_gate_evaluate() 한 곳에서만 한다.
#include <unity.h>
#include <cmath>
#include "modules/torque_vectoring.h"

// 모든 게이트 조건을 만족하는 입력. 각 테스트는 여기서 하나씩만 무너뜨린다.
static TVInput healthy() {
    TVInput in{};
    in.total_torque         = 20.0f;
    in.yaw_rate             = 0.0f;
    in.steering_angle       = Unit{};
    in.vehicle_speed        = 10.0f;
    in.ax                   = 0.0f;
    in.ay                   = 0.0f;
    in.dt                   = 0.01f;
    in.tv_enable_requested  = true;
    in.vehicle_speed_valid  = true;
    in.imu_valid            = true;
    return in;
}

// 기본 TV_PARAMS는 kp=ki=kd=0(마스터 OFF)이라 게인이 살아있는 사본이 필요하다.
static TVParams tuned() {
    TVParams p = TV_PARAMS;
    p.kp = 1.0f;
    return p;
}

void test_all_conditions_met_activates(void) {
    const TVGate g = tv_gate_evaluate(healthy(), tuned());
    TEST_ASSERT_TRUE(g.active);
}

void test_driver_switch_off_deactivates(void) {
    TVInput in = healthy();
    in.tv_enable_requested = false;
    const TVGate g = tv_gate_evaluate(in, tuned());
    TEST_ASSERT_FALSE(g.active);
    TEST_ASSERT_FALSE(g.driver_switch_on);
    TEST_ASSERT_TRUE(g.gains_enabled);   // 나머지 이유는 오염되지 않는다
    TEST_ASSERT_TRUE(g.imu_valid);
    TEST_ASSERT_TRUE(g.speed_valid);
    TEST_ASSERT_TRUE(g.speed_above_min);
}

void test_zero_gains_deactivates(void) {
    const TVGate g = tv_gate_evaluate(healthy(), TV_PARAMS);  // kp=ki=kd=0
    TEST_ASSERT_FALSE(g.active);
    TEST_ASSERT_FALSE(g.gains_enabled);
}

void test_any_single_nonzero_gain_enables_gains(void) {
    TVParams p = TV_PARAMS;
    p.kd = 0.5f;
    const TVGate g = tv_gate_evaluate(healthy(), p);
    TEST_ASSERT_TRUE(g.gains_enabled);
    TEST_ASSERT_TRUE(g.active);
}

void test_imu_invalid_deactivates(void) {
    TVInput in = healthy();
    in.imu_valid = false;
    const TVGate g = tv_gate_evaluate(in, tuned());
    TEST_ASSERT_FALSE(g.active);
    TEST_ASSERT_FALSE(g.imu_valid);
}

void test_speed_invalid_deactivates_even_at_healthy_speed(void) {
    // 이전 설계에서는 app_wiring이 !valid를 speed=0으로 밀수해서 "진짜 정지"와
    // "센서 사망"을 구분할 수 없었다. 이제 속도값이 멀쩡해도 플래그만으로 차단된다.
    TVInput in = healthy();
    in.vehicle_speed_valid = false;
    const TVGate g = tv_gate_evaluate(in, tuned());
    TEST_ASSERT_FALSE(g.active);
    TEST_ASSERT_FALSE(g.speed_valid);
    TEST_ASSERT_TRUE(g.speed_above_min);   // 속도 자체는 임계 위였다
}

void test_low_speed_deactivates(void) {
    TVInput in = healthy();
    in.vehicle_speed = 0.5f;   // tv_min_speed_mps = 1.0
    const TVGate g = tv_gate_evaluate(in, tuned());
    TEST_ASSERT_FALSE(g.active);
    TEST_ASSERT_FALSE(g.speed_above_min);
    TEST_ASSERT_TRUE(g.speed_valid);
}

void test_nan_speed_deactivates(void) {
    TVInput in = healthy();
    in.vehicle_speed = NAN;
    const TVGate g = tv_gate_evaluate(in, tuned());
    TEST_ASSERT_FALSE(g.active);
    TEST_ASSERT_FALSE(g.speed_above_min);
}

void test_reverse_speed_magnitude_counts(void) {
    // 차속 부호는 방향이고, TV 임계는 크기로 본다 (기존 |v| 검사 유지).
    TVInput in = healthy();
    in.vehicle_speed = -10.0f;
    const TVGate g = tv_gate_evaluate(in, tuned());
    TEST_ASSERT_TRUE(g.speed_above_min);
}

void test_multiple_failures_are_reported_independently(void) {
    TVInput in = healthy();
    in.imu_valid = false;
    in.vehicle_speed = 0.2f;
    const TVGate g = tv_gate_evaluate(in, TV_PARAMS);
    TEST_ASSERT_FALSE(g.active);
    TEST_ASSERT_FALSE(g.imu_valid);
    TEST_ASSERT_FALSE(g.speed_above_min);
    TEST_ASSERT_FALSE(g.gains_enabled);
    TEST_ASSERT_TRUE(g.driver_switch_on);
    TEST_ASSERT_TRUE(g.speed_valid);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_all_conditions_met_activates);
    RUN_TEST(test_driver_switch_off_deactivates);
    RUN_TEST(test_zero_gains_deactivates);
    RUN_TEST(test_any_single_nonzero_gain_enables_gains);
    RUN_TEST(test_imu_invalid_deactivates);
    RUN_TEST(test_speed_invalid_deactivates_even_at_healthy_speed);
    RUN_TEST(test_low_speed_deactivates);
    RUN_TEST(test_nan_speed_deactivates);
    RUN_TEST(test_reverse_speed_magnitude_counts);
    RUN_TEST(test_multiple_failures_are_reported_independently);
    return UNITY_END();
}
