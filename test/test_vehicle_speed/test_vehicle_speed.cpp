// 차속 추정 테스트
#include <unity.h>
#include "modules/vehicle_speed.h"
#include <cmath>

static const VehicleSpeedCalib CAL{};   // 실차 시험 시작값은 realcar_calibration.h에서 관리

// 차속[m/s] → 휠 rpm (테스트 입력 만들 때 쓰는 역환산)
static float mps_to_rpm(float mps) {
    return mps * 60.0f / (2.0f * 3.14159265f * CAL.tire_radius_m);
}

// 네 바퀴 모두 같은 속도, 모두 유효
static VehicleSpeedInput all_wheels(float mps) {
    VehicleSpeedInput in{};
    for (int i = 0; i < WHEEL_COUNT; ++i) {
        in.wheel_rpm[i] = Rpm(mps_to_rpm(mps));
        in.wheel_valid[i] = true;
    }
    return in;
}

void test_straight_uses_front_average(void) {
    VehicleSpeedState s{};
    VehicleSpeedOutput o = vehicle_speed_compute(all_wheels(10.0f), CAL, s);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, o.speed_mps);
}

void test_default_calibration_uses_rolling_tire_not_magnet_radius(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.2387f, CAL.tire_radius_m);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.140f, CAL.track_m);
}

void test_driven_wheel_spin_is_ignored(void) {
    // 후륜이 슬립해서 2배로 돌아도 차속 추정은 전륜만 본다.
    VehicleSpeedState s{};
    VehicleSpeedInput in = all_wheels(10.0f);
    in.wheel_rpm[WHEEL_RL] = Rpm(mps_to_rpm(20.0f));   // 휠스핀
    in.wheel_rpm[WHEEL_RR] = Rpm(mps_to_rpm(20.0f));
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
    VehicleSpeedInput in = all_wheels(v_cg);
    in.wheel_rpm[WHEEL_FL] = Rpm(mps_to_rpm(v_cg - yaw_term));
    in.wheel_rpm[WHEEL_FR] = Rpm(mps_to_rpm(v_cg + yaw_term));
    in.yaw_rate = yaw_dps;
    VehicleSpeedOutput o = vehicle_speed_compute(in, CAL, s);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, v_cg, o.speed_mps);

    // max(바깥 바퀴)를 썼다면 이만큼 과대평가됐을 것 — 그 값과는 달라야 한다.
    TEST_ASSERT_TRUE(std::fabs(o.speed_mps - (v_cg + yaw_term)) > 0.1f);
}

void test_single_invalid_front_wheel_is_yaw_corrected(void) {
    // 우전륜 샘플이 무효 → 좌전륜 + yaw 보정으로 CG 속도 복원.
    // 무효 바퀴의 rpm은 state에 남은 옛값이므로 쓰면 안 된다.
    const float v_cg = 10.0f;
    const float yaw_dps = 30.0f;
    const float yaw_term = (yaw_dps * 0.01745329f) * CAL.track_m * 0.5f;

    VehicleSpeedState s{};
    VehicleSpeedInput in = all_wheels(v_cg);
    in.wheel_rpm[WHEEL_FL] = Rpm(mps_to_rpm(v_cg - yaw_term));
    in.wheel_rpm[WHEEL_FR] = Rpm(mps_to_rpm(3.0f));     // 옛값
    in.wheel_valid[WHEEL_FR] = false;
    in.yaw_rate = yaw_dps;
    VehicleSpeedOutput o = vehicle_speed_compute(in, CAL, s);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, v_cg, o.speed_mps);
}

void test_both_front_invalid_holds_last_value(void) {
    // 전륜 둘 다 무효 → valid=false, 값은 직전 추정치 유지 (후륜으로 폴백하지 않는다).
    VehicleSpeedState s{};
    vehicle_speed_compute(all_wheels(10.0f), CAL, s);
    VehicleSpeedInput in = all_wheels(0.0f);
    in.wheel_rpm[WHEEL_RL] = Rpm(mps_to_rpm(15.0f));
    in.wheel_rpm[WHEEL_RR] = Rpm(mps_to_rpm(15.0f));
    in.wheel_valid[WHEEL_FL] = false;
    in.wheel_valid[WHEEL_FR] = false;
    VehicleSpeedOutput o = vehicle_speed_compute(in, CAL, s);
    TEST_ASSERT_FALSE(o.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 10.0f, o.speed_mps);
}

void test_fast_change_is_not_rejected(void) {
    // 샘플 간 변화량으로는 기각하지 않는다: 양자화로 튀어도 전륜이 유효하면 valid.
    VehicleSpeedState s{};
    vehicle_speed_compute(all_wheels(10.0f), CAL, s);
    VehicleSpeedOutput o = vehicle_speed_compute(all_wheels(11.0f), CAL, s);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 11.0f, o.speed_mps);
}

void test_standstill(void) {
    VehicleSpeedState s{};
    VehicleSpeedOutput o = vehicle_speed_compute(all_wheels(0.0f), CAL, s);
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, o.speed_mps);
}

void test_never_negative(void) {
    // 큰 yaw 보정이 들어가도 음수 차속은 나오지 않는다.
    VehicleSpeedState s{};
    VehicleSpeedInput in = all_wheels(0.0f);
    in.wheel_valid[WHEEL_FR] = false;
    in.yaw_rate = -300.0f;
    VehicleSpeedOutput o = vehicle_speed_compute(in, CAL, s);
    TEST_ASSERT_TRUE(o.speed_mps >= 0.0f);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_straight_uses_front_average);
    RUN_TEST(test_default_calibration_uses_rolling_tire_not_magnet_radius);
    RUN_TEST(test_driven_wheel_spin_is_ignored);
    RUN_TEST(test_cornering_yaw_component_cancels);
    RUN_TEST(test_single_invalid_front_wheel_is_yaw_corrected);
    RUN_TEST(test_both_front_invalid_holds_last_value);
    RUN_TEST(test_fast_change_is_not_rejected);
    RUN_TEST(test_standstill);
    RUN_TEST(test_never_negative);
    return UNITY_END();
}
