#include <unity.h>
#include <cmath>
#include "car_check_protocol.h"
#include "modules/car_check_status.h"
#include "modules/torque_vectoring.h"

void test_wire_golden_vectors() {
    uint8_t d[8]; car_check::Steering s; s.unit=-0.5f; s.valid=true; s.life=42;
    car_check::encode_steering(s,d);
    const uint8_t steer[8]={0x0c,0xfe,0,0,0,0,0x81,42};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(steer,d,8);
    car_check::Imu m; m.yaw_dps=-12.34f; m.ax_g=0.5f; m.ay_g=-1.25f;
    m.yaw_valid=m.accel_valid=true; m.life=42; car_check::encode_imu(m,d);
    const uint8_t imu[8]={0x2e,0xfb,0x32,0,0x83,0xff,0x83,42};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(imu,d,8);
    car_check::Wheels w; w.kph[0]=0; w.kph[1]=12.3f; w.kph[2]=45.6f;
    w.valid[0]=w.valid[1]=w.valid[2]=true; car_check::encode_wheels(w,d);
    const uint8_t wheels[8]={0,0,123,0,200,1,255,255};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(wheels,d,8);
}
void test_invalid_and_nonfinite_are_not_measurements() {
    uint8_t d[8]; car_check::Imu m; m.yaw_valid=m.accel_valid=true;
    m.yaw_dps=NAN; m.ax_g=INFINITY; car_check::encode_imu(m,d);
    TEST_ASSERT_EQUAL_UINT8(0x80,d[6]);
    for(int i=0;i<6;++i) TEST_ASSERT_EQUAL_UINT8(0,d[i]);
    car_check::Steering s; s.unit=2; s.valid=true; car_check::encode_steering(s,d);
    TEST_ASSERT_EQUAL_UINT8(0x80,d[6]);
    car_check::Wheels w;
    for(int i=0;i<4;++i) w.valid[i]=true;
    w.kph[0]=NAN; w.kph[1]=-1; w.kph[2]=INFINITY; w.kph[3]=7000;
    car_check::encode_wheels(w,d);
    for(auto v:d) TEST_ASSERT_EQUAL_UINT8(255,v);
}
void test_requested_is_not_active() {
    CarCheckStatusInput i{};
    i.tv_requested=i.regen_requested=i.cluster_fresh=true;
    const auto o=car_check_status_compute(i);
    TEST_ASSERT_TRUE(o.tv_requested); TEST_ASSERT_FALSE(o.tv_active);
    TEST_ASSERT_FALSE(o.regen_available); TEST_ASSERT_FALSE(o.regen_active);
    TEST_ASSERT_TRUE(o.tv_block & car_check::TV_GAINS_ZERO);
    TEST_ASSERT_TRUE(o.regen_block & car_check::REGEN_NOT_VALIDATED);
    TEST_ASSERT_TRUE(o.regen_block & car_check::REGEN_NO_BRAKE_SENSOR);
    uint8_t d[8]; car_check::encode_control(o,d);
    TEST_ASSERT_EQUAL_UINT8(1,d[0]); TEST_ASSERT_EQUAL_UINT8(3,d[1]);
    TEST_ASSERT_EQUAL_UINT8(32,d[2]);
}
void test_regen_observation_checks_sign_and_freshness() {
    CarCheckStatusInput i{};
    i.regen_requested=i.cluster_fresh=i.output_allowed=i.regen_validated=true;
    i.brake_installed=i.bms_valid=i.brake_demand=i.longitudinal_regen_demand=true;
    i.pack_soc=0.5f; i.direction_sign=1; i.left_a=i.right_a=-10;
    TEST_ASSERT_TRUE(car_check_status_compute(i).regen_active);
    i.left_a=i.right_a=10;
    auto o=car_check_status_compute(i);
    TEST_ASSERT_FALSE(o.regen_active);
    TEST_ASSERT_TRUE(o.regen_block & car_check::REGEN_DIRECTION_MISMATCH);
    i.direction_sign=-1; TEST_ASSERT_TRUE(car_check_status_compute(i).regen_active);
    i.bms_valid=false; TEST_ASSERT_FALSE(car_check_status_compute(i).regen_active);
    i.bms_valid=true; i.pack_soc=NAN;
    TEST_ASSERT_FALSE(car_check_status_compute(i).regen_available);
    i.pack_soc=0.5f; i.test_override=true;
    TEST_ASSERT_FALSE(car_check_status_compute(i).regen_active);
}
void test_tv_pipeline_flag_and_override() {
    TVYawState state{};
    const TVInput i{100,0,0,10,0,0,0.01f,true};
    TEST_ASSERT_FALSE(tv_compute(i,state).control_active); // real defaults: zero gains
    CarCheckStatusInput c{};
    c.tv_pipeline_active=c.output_allowed=c.cluster_fresh=true;
    TEST_ASSERT_TRUE(car_check_status_compute(c).tv_active);
    c.test_override=true; TEST_ASSERT_FALSE(car_check_status_compute(c).tv_active);
}
void test_wheel_validity_does_not_claim_cable_health() {
    TEST_ASSERT_TRUE(car_check_wheel_sample_valid(true,0,10,24));
    TEST_ASSERT_TRUE(car_check_wheel_sample_valid(true,4,10,24));
    TEST_ASSERT_FALSE(car_check_wheel_sample_valid(true,0xffff8000U,10,24));
    TEST_ASSERT_FALSE(car_check_wheel_sample_valid(false,0,10,24));
    TEST_ASSERT_FALSE(car_check_wheel_sample_valid(true,1,0,24));
    TEST_ASSERT_FALSE(car_check_wheel_sample_valid(true,1,101,24));
    TEST_ASSERT_FALSE(car_check_wheel_sample_valid(true,1,10,0));
}
void setUp() {} void tearDown() {}
int main(int,char**) { UNITY_BEGIN();
    RUN_TEST(test_wire_golden_vectors);
    RUN_TEST(test_invalid_and_nonfinite_are_not_measurements);
    RUN_TEST(test_requested_is_not_active);
    RUN_TEST(test_regen_observation_checks_sign_and_freshness);
    RUN_TEST(test_tv_pipeline_flag_and_override);
    RUN_TEST(test_wheel_validity_does_not_claim_cable_health);
    return UNITY_END(); }
