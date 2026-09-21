#include <unity.h>
#include <cmath>
#include "energy_meter.h"
#include "modules/drive_supervisor.h"

void setUp() {}
void tearDown() {}
// Cluster golden fixture: HV 50 V, I -10 A, LV 12.34 V, CPU -5 C.
const uint8_t charge[8] = {0xF4,0x01,0x9C,0xFF,0xD2,0x04,0x0C,0xFE};

void test_cluster_signed_layout() {
    TEST_ASSERT_EQUAL_HEX32(0x1CF5FFC1, CAN_ID_EM_RECORD);
    TEST_ASSERT_EQUAL_HEX32(0x1CF6FFC1, CAN_ID_EM_SYNC);
    const auto em=decode_em_record(charge);
    TEST_ASSERT_FLOAT_WITHIN(0.001f,50,em.hv_voltage_v);
    TEST_ASSERT_FLOAT_WITHIN(0.001f,-10,em.current_a);
    TEST_ASSERT_FLOAT_WITHIN(0.001f,12.34f,em.lv_voltage_v);
    TEST_ASSERT_FLOAT_WITHIN(0.001f,-5,em.cpu_temperature_c);
    TEST_ASSERT_FLOAT_WITHIN(0.001f,-500,em.power_w);
    const uint8_t discharge[8]={0xF4,1,0xD0,7,0,0,0,0}; // 200A, 10kW
    TEST_ASSERT_FLOAT_WITHIN(0.01f,10000,decode_em_record(discharge).power_w);
    const uint8_t negative_hv[8]={0xFF,0xFF,0,0,0,0,0,0};
    TEST_ASSERT_FLOAT_WITHIN(0.001f,-0.1f,decode_em_record(negative_hv).hv_voltage_v);
}

void test_frame_validation() {
    EnergyMeterReceiver r;
    TEST_ASSERT_FALSE(r.fresh(0,100));
    TEST_ASSERT_FALSE(r.receive(0x180117D0,true,false,8,charge,0));
    TEST_ASSERT_FALSE(r.receive(CAN_ID_EM_SYNC,true,false,8,charge,0));
    TEST_ASSERT_FALSE(r.seen);
    TEST_ASSERT_TRUE(r.receive(CAN_ID_EM_RECORD,true,false,8,charge,0));
    TEST_ASSERT_TRUE(r.fresh(0,100)); // boot timestamp zero is not 'never seen'
    TEST_ASSERT_TRUE(r.receive(CAN_ID_EM_RECORD,true,true,8,charge,1));
    TEST_ASSERT_FALSE(r.fresh(1,100));
    r.receive(CAN_ID_EM_RECORD,false,false,8,charge,2);
    r.receive(CAN_ID_EM_RECORD,true,false,7,charge,3);
    r.receive(CAN_ID_EM_RECORD,true,false,8,nullptr,4);
    TEST_ASSERT_EQUAL_UINT32(1,r.rx_count);
    TEST_ASSERT_EQUAL_UINT32(4,r.rejected_count);
    TEST_ASSERT_EQUAL_UINT32(0,r.last_rx_ms);
}

void test_stale_recovery_rollover_and_zero_hv() {
    EnergyMeterReceiver r;
    r.receive(CAN_ID_EM_RECORD,true,false,8,charge,0xFFFFFFF0u);
    TEST_ASSERT_TRUE(r.fresh(0x54u,100));
    TEST_ASSERT_FALSE(r.fresh(0x55u,100));
    r.receive(CAN_ID_EM_SYNC,true,false,8,charge,0x55);
    TEST_ASSERT_FALSE(r.fresh(0x55u,100));
    r.receive(CAN_ID_EM_RECORD,true,false,8,charge,0x55);
    TEST_ASSERT_TRUE(r.fresh(0x55u,100));
    const uint8_t zero[8]={};
    r.receive(CAN_ID_EM_RECORD,true,false,8,zero,100);
    TEST_ASSERT_TRUE(r.seen);
    TEST_ASSERT_EQUAL_FLOAT(0,r.record.hv_voltage_v);
    TEST_ASSERT_FALSE(r.fresh(100,100));
}

DriveSupervisorParams params() {
    DriveSupervisorParams p{};
    p.power_soft_limit_w=8000;
    p.controller_derate_start_c=p.motor_derate_start_c=70;
    p.controller_cutoff_c=p.motor_cutoff_c=90;
    p.enable_energy_meter_limit=true;
    return p;
}
DriveSupervisorInput input() {
    DriveSupervisorInput i{};
    i.controller_feedback_fresh=true;
    i.propulsion_requested=true;
    i.requested_left_a=i.requested_right_a=100;
    i.energy_meter_valid=true;
    i.energy_meter_power_w=10000;
    return i;
}
void test_meter_governs_and_observe_only_does_not() {
    auto p=params(); auto i=input(); DriveSupervisorState s{};
    auto o=drive_supervisor_compute(i,p,s);
    TEST_ASSERT_FLOAT_WITHIN(0.001f,80,o.left_a);
    TEST_ASSERT_FLOAT_WITHIN(0.001f,80,o.right_a);
    TEST_ASSERT_TRUE(o.power_limited);
    p.enable_energy_meter_limit=false;
    TEST_ASSERT_EQUAL_FLOAT(100,drive_supervisor_compute(i,p,s).left_a);
    i.energy_meter_valid=false;
    TEST_ASSERT_EQUAL_FLOAT(100,drive_supervisor_compute(i,p,s).left_a);
}
void test_missing_meter_blocks_forward_and_reverse_and_recovers_with_ramp() {
    auto p=params(); auto i=input(); DriveSupervisorState s{};
    i.energy_meter_valid=false;
    TEST_ASSERT_TRUE(drive_supervisor_compute(i,p,s).energy_meter_blocked);
    TEST_ASSERT_EQUAL_FLOAT(0,drive_supervisor_compute(i,p,s).left_a);
    i.requested_left_a=i.requested_right_a=-100;
    TEST_ASSERT_EQUAL_FLOAT(0,drive_supervisor_compute(i,p,s).left_a);
    i.energy_meter_valid=true; i.energy_meter_power_w=NAN;
    TEST_ASSERT_TRUE(drive_supervisor_compute(i,p,s).energy_meter_blocked);
    i.energy_meter_power_w=0;
    p.drive_current_max_per_motor_a=500; p.drive_current_rise_time_s=0.5f;
    i.control_dt_s=0.01f;
    TEST_ASSERT_FLOAT_WITHIN(0.001f,-10,drive_supervisor_compute(i,p,s).left_a);
}
void test_charge_is_not_discharge_and_model_guards_remain() {
    auto p=params(); auto i=input(); DriveSupervisorState s{};
    i.energy_meter_power_w=-10000;
    TEST_ASSERT_EQUAL_FLOAT(100,drive_supervisor_compute(i,p,s).left_a);
    i.bus_voltage_left_v=50; i.bus_current_left_a=200;
    TEST_ASSERT_FLOAT_WITHIN(0.001f,80,drive_supervisor_compute(i,p,s).left_a);
    i.propulsion_requested=false;
    i.requested_left_a=i.requested_right_a=-10;
    i.energy_meter_valid=false;
    TEST_ASSERT_EQUAL_FLOAT(-10,drive_supervisor_compute(i,p,s).left_a);
}
int main(int,char**) {
    UNITY_BEGIN();
    RUN_TEST(test_cluster_signed_layout);
    RUN_TEST(test_frame_validation);
    RUN_TEST(test_stale_recovery_rollover_and_zero_hv);
    RUN_TEST(test_meter_governs_and_observe_only_does_not);
    RUN_TEST(test_missing_meter_blocks_forward_and_reverse_and_recovers_with_ramp);
    RUN_TEST(test_charge_is_not_discharge_and_model_guards_remain);
    return UNITY_END();
}
