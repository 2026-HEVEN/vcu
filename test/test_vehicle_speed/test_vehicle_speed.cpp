// 차속 추정 테스트
#include <unity.h>
#include "modules/vehicle_speed.h"
#include <cmath>

static const VehicleSpeedCalib CAL{};   // r=0.165m, track=1.20m, a_max=15 m/s^2

// 차속[m/s] → 휠 rpm (테스트 입력 만들 때 쓰는 역환산)
static float mps_to_rpm(float mps) {
    return mps * 60.0f / (2.0f * 3.14159265f * CAL.tire_radius_m);
}

// 상태를 원하는 속도로 미리 안정화시킨다 (급변 검사가 걸리지 않도록)
static void prime(VehicleSpeedState &s, float mps) {
    VehicleSpeedInput in{};
    for (int i = 0; i < WHEEL_COUNT; ++i) in.wheel_rpm[i] = Rpm(mps_to_rpm(mps));
    in.yaw_rate = 0.0f; in.dt = 0.01f;
    for (int k = 0; k < 50; ++k) vehicle_speed_compute(in, CAL, s);
}

void test_straight_uses_front_average(void) {
    VehicleSpeedState s{};
    VehicleSpeedInput in{};
    for (int i = 0; i < WHEEL_COUNT; ++i) in.wheel_rpm[i] = Rpm(mps_to_rpm(10.0f));
    in.dt = 0.01f;
    VehicleSpeedOutput o = vehicle_speed_compute(in, CAL, s);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, o.speed_mps);
}

void test_driven_wheel_spin_is_ignored(void) {
    // 후륜이 슬립해서 2배로 돌아도 차속 추정은 전륜만 본다.
    VehicleSpeedState s{};
    prime(s, 10.0f);
    VehicleSpeedInput in{};
    in.wheel_rpm[WHEEL_FL] = Rpm(mps_to_rpm(10.0f));
    in.wheel_rpm[WHEEL_FR] = Rpm(mps_to_rpm(10.0f));
    in.wheel_rpm[WHEEL_RL] = Rpm(mps_to_rpm(20.0f));   // 휠스핀
    in.wheel_rpm[WHEEL_RR] = Rpm(mps_to_rpm(20.0f));
    in.dt = 0.01f;
    VehicleSpeedOutput o = vehicle_speed_compute(in, CAL, s);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, o.speed_mps);
}

void test_cornering_yaw_component_cancels(void) {
    // 좌선회: 좌전륜이 안쪽(느림), 우전륜이 바깥(빠름).
    // 평균을 쓰면 CG 속도가 그대로 나와야 한다 (max를 쓰면 과대평가된다).
    const float v_cg = 10.0f;
    const float yaw_dps = 30.0f;
    const float yaw_term = (yaw_dps * 0.01745329f) * CAL.track_m * 0.5f;

    VehicleSpeedState s{};
    prime(s, v_cg);
    VehicleSpeedInput in{};
    in.wheel_rpm[WHEEL_FL] = Rpm(mps_to_rpm(v_cg - yaw_term));
    in.wheel_rpm[WHEEL_FR] = Rpm(mps_to_rpm(v_cg + yaw_term));
    in.wheel_rpm[WHEEL_RL] = Rpm(mps_to_rpm(v_cg));
    in.wheel_rpm[WHEEL_RR] = Rpm(mps_to_rpm(v_cg));
    in.yaw_rate = yaw_dps;
    in.dt = 0.01f;
    VehicleSpeedOutput o = vehicle_speed_compute(in, CAL, s);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, v_cg, o.speed_mps);

    // max(바깥 바퀴)를 썼다면 이만큼 과대평가됐을 것 — 그 값과는 달라야 한다.
    TEST_ASSERT_TRUE(std::fabs(o.speed_mps - (v_cg + yaw_term)) > 0.1f);
}

void test_single_front_wheel_dropout_is_yaw_corrected(void) {
    // 우전륜 신호가 끊겨 0으로 떨어짐 → 좌전륜 + yaw 보정으로 CG 속도 복원.
    const float v_cg = 10.0f;
    const float yaw_dps = 30.0f;
    const float yaw_term = (yaw_dps * 0.01745329f) * CAL.track_m * 0.5f;

    VehicleSpeedState s{};
    prime(s, v_cg);
    VehicleSpeedInput in{};
    in.wheel_rpm[WHEEL_FL] = Rpm(mps_to_rpm(v_cg - yaw_term));
    in.wheel_rpm[WHEEL_FR] = Rpm(0.0f);                 // 드롭아웃
    in.wheel_rpm[WHEEL_RL] = Rpm(mps_to_rpm(v_cg));
    in.wheel_rpm[WHEEL_RR] = Rpm(mps_to_rpm(v_cg));
    in.yaw_rate = yaw_dps;
    in.dt = 0.01f;
    VehicleSpeedOutput o = vehicle_speed_compute(in, CAL, s);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, v_cg, o.speed_mps);
}

void test_both_front_lost_is_invalid(void) {
    // 전륜 둘 다 급변(락업/단선) → valid=false, 값은 후륜 폴백.
    VehicleSpeedState s{};
    prime(s, 10.0f);
    VehicleSpeedInput in{};
    in.wheel_rpm[WHEEL_FL] = Rpm(0.0f);
    in.wheel_rpm[WHEEL_FR] = Rpm(0.0f);
    in.wheel_rpm[WHEEL_RL] = Rpm(mps_to_rpm(9.0f));
    in.wheel_rpm[WHEEL_RR] = Rpm(mps_to_rpm(9.0f));
    in.dt = 0.01f;
    VehicleSpeedOutput o = vehicle_speed_compute(in, CAL, s);
    TEST_ASSERT_FALSE(o.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 9.0f, o.speed_mps);
}

void test_impossible_jump_is_rejected(void) {
    // 10 m/s에서 한 tick(10ms) 만에 30 m/s는 물리적으로 불가 (a_max=15 → 0.15 m/s).
    VehicleSpeedState s{};
    prime(s, 10.0f);
    VehicleSpeedInput in{};
    for (int i = 0; i < WHEEL_COUNT; ++i) in.wheel_rpm[i] = Rpm(mps_to_rpm(30.0f));
    in.dt = 0.01f;
    VehicleSpeedOutput o = vehicle_speed_compute(in, CAL, s);
    TEST_ASSERT_FALSE(o.valid);          // 전륜 둘 다 기각
    TEST_ASSERT_TRUE(o.speed_mps < 31.0f);
}

void test_gradual_acceleration_is_tracked(void) {
    // 물리적으로 가능한 가속(10 m/s^2)은 정상 추종된다.
    VehicleSpeedState s{};
    prime(s, 5.0f);
    float v = 5.0f;
    for (int k = 0; k < 100; ++k) {       // 1초간 10 m/s^2
        v += 10.0f * 0.01f;
        VehicleSpeedInput in{};
        for (int i = 0; i < WHEEL_COUNT; ++i) in.wheel_rpm[i] = Rpm(mps_to_rpm(v));
        in.dt = 0.01f;
        VehicleSpeedOutput o = vehicle_speed_compute(in, CAL, s);
        TEST_ASSERT_TRUE(o.valid);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 15.0f, s.speed_mps);
}

void test_standstill(void) {
    VehicleSpeedState s{};
    VehicleSpeedInput in{};
    in.dt = 0.01f;
    VehicleSpeedOutput o = vehicle_speed_compute(in, CAL, s);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, o.speed_mps);
}

void test_never_negative(void) {
    // 큰 yaw 보정이 들어가도 음수 차속은 나오지 않는다.
    VehicleSpeedState s{};
    VehicleSpeedInput in{};
    in.yaw_rate = -300.0f;
    in.dt = 0.01f;
    VehicleSpeedOutput o = vehicle_speed_compute(in, CAL, s);
    TEST_ASSERT_TRUE(o.speed_mps >= 0.0f);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_straight_uses_front_average);
    RUN_TEST(test_driven_wheel_spin_is_ignored);
    RUN_TEST(test_cornering_yaw_component_cancels);
    RUN_TEST(test_single_front_wheel_dropout_is_yaw_corrected);
    RUN_TEST(test_both_front_lost_is_invalid);
    RUN_TEST(test_impossible_jump_is_rejected);
    RUN_TEST(test_gradual_acceleration_is_tracked);
    RUN_TEST(test_standstill);
    RUN_TEST(test_never_negative);
    return UNITY_END();
}
