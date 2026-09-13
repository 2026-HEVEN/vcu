#include <unity.h>
#include "can_protocol.h"
#include <cmath>

void test_zero_amps_offset(void)   { TEST_ASSERT_EQUAL_UINT16(32000, torque_to_raw(0.0f)); }
void test_positive_amps(void)      { TEST_ASSERT_EQUAL_UINT16(32320, torque_to_raw(32.0f)); }
void test_negative_regen(void)     { TEST_ASSERT_EQUAL_UINT16(31680, torque_to_raw(-32.0f)); }
void test_roundtrip(void)          { TEST_ASSERT_FLOAT_WITHIN(0.01f, 17.5f, raw_to_torque(torque_to_raw(17.5f))); }
void test_motor_speed_offset(void) {
    TEST_ASSERT_EQUAL_UINT16(32000, motor_speed_to_raw(0));
    TEST_ASSERT_EQUAL_UINT16(36000, motor_speed_to_raw(4000));
}

void test_encode_drive_control_frame(void) {
    uint8_t out[8];
    encode_motor_control(15.0f, 4000, true, 0x5A, out);
    TEST_ASSERT_EQUAL_UINT16(32150, (uint16_t)(out[0] | (out[1] << 8)));
    TEST_ASSERT_EQUAL_UINT16(36000, (uint16_t)(out[2] | (out[3] << 8)));
    TEST_ASSERT_EQUAL_UINT8(0x01, out[4]);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[5]);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[6]);
    TEST_ASSERT_EQUAL_UINT8(0x5A, out[7]);
}

void test_encode_regen_control_frame(void) {
    uint8_t out[8];
    encode_motor_control(-20.0f, 0, true, 7, out);
    TEST_ASSERT_EQUAL_UINT16(31800, (uint16_t)(out[0] | (out[1] << 8)));
    TEST_ASSERT_EQUAL_UINT16(32000, (uint16_t)(out[2] | (out[3] << 8)));
    TEST_ASSERT_EQUAL_UINT8(0x01, out[4]);
    TEST_ASSERT_EQUAL_UINT8(7, out[7]);
}

void test_encode_halted_zero_frame(void) {
    uint8_t out[8];
    encode_motor_control(0.0f, 0, false, 9, out);
    TEST_ASSERT_EQUAL_UINT16(32000, (uint16_t)(out[0] | (out[1] << 8)));
    TEST_ASSERT_EQUAL_UINT16(32000, (uint16_t)(out[2] | (out[3] << 8)));
    TEST_ASSERT_EQUAL_UINT8(0x00, out[4]);
    TEST_ASSERT_EQUAL_UINT8(9, out[7]);
}
void test_ids(void) {
    TEST_ASSERT_EQUAL_HEX32(0x0C01EFD0, CAN_ID_TORQUE_L);
    TEST_ASSERT_EQUAL_HEX32(0x0C01F0D0, CAN_ID_TORQUE_R);
    TEST_ASSERT_EQUAL_HEX32(0x1801D0EF, CAN_ID_FB1_L);
    TEST_ASSERT_EQUAL_HEX32(0x1802D0EF, CAN_ID_FB2_L);
    TEST_ASSERT_EQUAL_HEX32(0x1801D0F0, CAN_ID_FB1_R);
    TEST_ASSERT_EQUAL_HEX32(0x1802D0F0, CAN_ID_FB2_R);
    TEST_ASSERT_EQUAL_HEX32(0x1801D0C0, CAN_ID_CLUSTER_CMD);
    TEST_ASSERT_EQUAL_HEX32(0x1801C0D0, CAN_ID_VCU_CLUSTER_STATUS);
    TEST_ASSERT_EQUAL_HEX32(0x1803C0D0, CAN_ID_VCU_VEHICLE_SPEED);
    TEST_ASSERT_EQUAL_HEX32(0x1804C0D0, CAN_ID_VCU_STEERING);
    TEST_ASSERT_EQUAL_HEX32(0x1805C0D0, CAN_ID_VCU_IMU);
    TEST_ASSERT_EQUAL_HEX32(0x18F3FFC0, CAN_ID_CLUSTER_BMS_STATUS);
}

void test_decode_controller_feedback(void) {
    uint8_t part1[8] = {0x3A,0x02, 0x16,0x7D, 0xF0,0x7D, 0xB8,0x0B};
    ControllerFeedbackPart1 fb1 = decode_controller_feedback_part1(part1);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 57.0f, fb1.bus_voltage_v);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.2f, fb1.bus_current_a);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 24.0f, fb1.phase_current_a);
    TEST_ASSERT_EQUAL_INT(-29000, fb1.motor_speed_rpm);

    uint8_t part2[8] = {80, 90, 0x01, 0x01, 0x02, 0x04, 0, 9};
    ControllerFeedbackPart2 fb2 = decode_controller_feedback_part2(part2);
    TEST_ASSERT_EQUAL_INT(40, fb2.controller_temp_c);
    TEST_ASSERT_EQUAL_INT(50, fb2.motor_temp_c);
    TEST_ASSERT_TRUE(fb2.running);
    TEST_ASSERT_TRUE(fb2.any_fault());
    TEST_ASSERT_EQUAL_UINT8(9, fb2.life);
}

void test_encode_vcu_cluster_status(void) {
    uint8_t out[8];
    encode_vcu_cluster_status(2, true, true, true, false, 88,
                              true, 60, 0x5A, out);
    TEST_ASSERT_EQUAL_UINT8(2, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x1B, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0, out[2]);
    TEST_ASSERT_EQUAL_UINT8(60, out[3]);
    TEST_ASSERT_EQUAL_UINT8(0x5A, out[7]);
}

void test_encode_vcu_cluster_status_clears_invalid_throttle(void) {
    uint8_t out[8];
    encode_vcu_cluster_status(0, false, false, false, false, 0,
                              false, 87, 0x2A, out);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[1] & 0x08);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[1] & 0x10);
    TEST_ASSERT_EQUAL_UINT8(0, out[3]);
}

void test_sensor_telemetry_encoders(void) {
    uint8_t out[8];
    encode_vcu_steering(-0.25f, out);
    TEST_ASSERT_EQUAL_INT16(-250, (int16_t)(out[0] | (out[1] << 8)));
    encode_vcu_imu(12.34f, -0.5f, 1.25f, out);
    TEST_ASSERT_EQUAL_INT16(1234, (int16_t)(out[0] | (out[1] << 8)));
    TEST_ASSERT_EQUAL_INT16(-50, (int16_t)(out[2] | (out[3] << 8)));
    TEST_ASSERT_EQUAL_INT16(125, (int16_t)(out[4] | (out[5] << 8)));
}

void test_decode_cluster_bms_status_is_diagnostic(void) {
    uint8_t data[8] = {0x03, 78, 0x00,0x02, 0x05,0x7D, 75, 4};
    ClusterBmsStatus bms = decode_cluster_bms_status(data);
    TEST_ASSERT_TRUE(bms.valid);
    TEST_ASSERT_TRUE(bms.ble_connected);
    TEST_ASSERT_EQUAL_UINT8(78, bms.soc_pct);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 51.2f, bms.pack_voltage_v);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, bms.pack_current_a);
    TEST_ASSERT_EQUAL_INT(35, bms.temperature_c);
}

void test_decode_cluster_command_bits(void) {
    uint8_t data[8] = {};
    data[1] = 0x0B; // TC + Regen Auto + Debug, bit2 reserved clear
    data[2] = 0x01; // Paddock

    ClusterCommandRequest cmd = decode_cluster_command(data);
    TEST_ASSERT_TRUE(cmd.tv_enabled);
    TEST_ASSERT_TRUE(cmd.regen_auto_enabled);
    TEST_ASSERT_TRUE(cmd.debug_enabled);
    TEST_ASSERT_TRUE(cmd.paddock_request);
}

void test_decode_cluster_command_regen_off(void) {
    uint8_t data[8] = {};
    data[1] = 0x04; // reserved bit must not imply regen auto
    ClusterCommandRequest cmd = decode_cluster_command(data);
    TEST_ASSERT_FALSE(cmd.tv_enabled);
    TEST_ASSERT_FALSE(cmd.regen_auto_enabled);
    TEST_ASSERT_FALSE(cmd.debug_enabled);
    TEST_ASSERT_FALSE(cmd.paddock_request);
}

void test_vehicle_speed_kph_to_raw_clamps_and_rounds(void) {
    TEST_ASSERT_EQUAL_UINT16(0, vehicle_speed_kph_to_raw(-1.0f));
    TEST_ASSERT_EQUAL_UINT16(563, vehicle_speed_kph_to_raw(56.3f));
    TEST_ASSERT_EQUAL_UINT16(564, vehicle_speed_kph_to_raw(56.35f));
    TEST_ASSERT_EQUAL_UINT16(65535, vehicle_speed_kph_to_raw(7000.0f));
}

void test_encode_vcu_vehicle_speed(void) {
    uint8_t out[8];
    encode_vcu_vehicle_speed(56.3f, true, out);
    TEST_ASSERT_EQUAL_UINT8(0x33, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x02, out[1]);
    TEST_ASSERT_EQUAL_UINT8(1, out[2]);
    for (int i = 3; i < 8; ++i) TEST_ASSERT_EQUAL_UINT8(0, out[i]);

    encode_vcu_vehicle_speed(56.3f, false, out);
    for (int i = 0; i < 8; ++i) TEST_ASSERT_EQUAL_UINT8(0, out[i]);
}

// --- motor_command_resolve: 좌·우 명령 확정 (M2) ---

// 전부 허용, Drive, 좌우 100 A. 각 테스트는 여기서 한 가지만 바꾼다.
static MotorCommandSnapshot mc_snapshot(void) {
    MotorCommandSnapshot s;
    s.seq = 42;
    s.published_ms = 1000;
    s.left_a = 100.0f;
    s.right_a = 100.0f;
    s.gear = Gear::Drive;
    s.safety_allow = true;
    s.throttle_signal_valid = true;
    s.propulsion_direction_armed = true;
    return s;
}

static MotorCommandGates mc_gates(void) {
    MotorCommandGates g;
    g.scheduler_alive = true;
    g.reconnect_inhibit = false;
    g.component_test_inhibit = false;
    g.snapshot_fresh = true;
    g.reconnect_ramp_scale = 1.0f;
    return g;
}

static MotorFrameCommand mc_resolve(const MotorCommandSnapshot &s,
                                    const MotorCommandGates &g) {
    return motor_command_resolve(s, g);
}

// 불허 결과는 항상 좌우 동일하게 완전 차단이어야 한다.
static void mc_assert_both_blocked(const MotorFrameCommand &cmd) {
    TEST_ASSERT_FALSE(cmd.normal_allow);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.left_a);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.right_a);
    TEST_ASSERT_FALSE(cmd.run_L);
    TEST_ASSERT_FALSE(cmd.run_R);
    TEST_ASSERT_EQUAL_INT(0, cmd.target_rpm_L);
    TEST_ASSERT_EQUAL_INT(0, cmd.target_rpm_R);
}

void test_mc_drive_commands_both_motors_forward(void) {
    const MotorFrameCommand cmd = mc_resolve(mc_snapshot(), mc_gates());
    TEST_ASSERT_TRUE(cmd.normal_allow);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cmd.left_a);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cmd.right_a);
    TEST_ASSERT_TRUE(cmd.run_L);
    TEST_ASSERT_TRUE(cmd.run_R);
    TEST_ASSERT_EQUAL_INT(4000, cmd.target_rpm_L);
    TEST_ASSERT_EQUAL_INT(4000, cmd.target_rpm_R);
}

void test_mc_reverse_commands_both_motors_backward(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.gear = Gear::Reverse;
    const MotorFrameCommand cmd = mc_resolve(s, mc_gates());
    TEST_ASSERT_TRUE(cmd.normal_allow);
    TEST_ASSERT_EQUAL_INT(-4000, cmd.target_rpm_L);
    TEST_ASSERT_EQUAL_INT(-4000, cmd.target_rpm_R);
}

void test_mc_regen_in_drive_targets_zero_rpm(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.left_a = -20.0f;
    s.right_a = -20.0f;
    const MotorFrameCommand cmd = mc_resolve(s, mc_gates());
    TEST_ASSERT_TRUE(cmd.normal_allow);
    TEST_ASSERT_EQUAL_INT(0, cmd.target_rpm_L);
    TEST_ASSERT_EQUAL_INT(0, cmd.target_rpm_R);
}

void test_mc_safety_halt_blocks_both(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.safety_allow = false;
    mc_assert_both_blocked(mc_resolve(s, mc_gates()));
}

void test_mc_non_propulsion_gear_blocks_both(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.gear = Gear::Neutral;
    mc_assert_both_blocked(mc_resolve(s, mc_gates()));
    s.gear = Gear::Park;
    mc_assert_both_blocked(mc_resolve(s, mc_gates()));
}

void test_mc_stale_snapshot_blocks_both(void) {
    MotorCommandGates g = mc_gates();
    g.snapshot_fresh = false;
    mc_assert_both_blocked(mc_resolve(mc_snapshot(), g));
}

void test_mc_reconnect_ramp_scales_both_sides_equally(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.left_a = 300.0f;
    s.right_a = 200.0f;
    MotorCommandGates g = mc_gates();
    g.reconnect_ramp_scale = 0.5f;
    const MotorFrameCommand cmd = mc_resolve(s, g);
    TEST_ASSERT_EQUAL_FLOAT(150.0f, cmd.left_a);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cmd.right_a);
    // 램프는 비율이므로 좌우 비가 보존되어야 한다.
    TEST_ASSERT_EQUAL_FLOAT(s.left_a / s.right_a, cmd.left_a / cmd.right_a);
}

// M2 지적 ②의 회귀 방지. 좌우 전류 크기가 달라도 gear가 하나이므로
// 목표 rpm 부호는 절대 갈릴 수 없다.
void test_mc_asymmetric_current_never_splits_direction(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.left_a = 300.0f;
    s.right_a = 5.0f;
    MotorFrameCommand cmd = mc_resolve(s, mc_gates());
    TEST_ASSERT_EQUAL_INT(cmd.target_rpm_L, cmd.target_rpm_R);
    TEST_ASSERT_EQUAL_INT(4000, cmd.target_rpm_L);

    s.gear = Gear::Reverse;
    cmd = mc_resolve(s, mc_gates());
    TEST_ASSERT_EQUAL_INT(cmd.target_rpm_L, cmd.target_rpm_R);
    TEST_ASSERT_EQUAL_INT(-4000, cmd.target_rpm_L);
}

// NaN은 Clamped<> 도메인 타입을 그대로 통과한다. 한쪽만 NaN이어도
// 양쪽을 함께 차단해야 한다. 한쪽만 0으로 만들면 그것이 곧 좌우 비대칭이다.
void test_mc_non_finite_command_blocks_both_sides(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.left_a = std::nanf("");
    mc_assert_both_blocked(mc_resolve(s, mc_gates()));

    s = mc_snapshot();
    s.right_a = INFINITY;
    mc_assert_both_blocked(mc_resolve(s, mc_gates()));
}

void test_mc_direction_not_armed_blocks_both(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.propulsion_direction_armed = false;
    mc_assert_both_blocked(mc_resolve(s, mc_gates()));
}

void test_mc_invalid_throttle_signal_blocks_both(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.throttle_signal_valid = false;
    mc_assert_both_blocked(mc_resolve(s, mc_gates()));
}

void test_mc_core1_gates_block_both(void) {
    MotorCommandGates g = mc_gates();
    g.scheduler_alive = false;
    mc_assert_both_blocked(mc_resolve(mc_snapshot(), g));

    g = mc_gates();
    g.reconnect_inhibit = true;
    mc_assert_both_blocked(mc_resolve(mc_snapshot(), g));

    g = mc_gates();
    g.component_test_inhibit = true;
    mc_assert_both_blocked(mc_resolve(mc_snapshot(), g));
}

// 램프 스케일 자체가 오염돼도 명령을 키우거나 NaN을 내보내면 안 된다.
void test_mc_ramp_scale_is_clamped_and_nan_safe(void) {
    MotorCommandGates g = mc_gates();
    g.reconnect_ramp_scale = 2.0f;
    TEST_ASSERT_EQUAL_FLOAT(100.0f, mc_resolve(mc_snapshot(), g).left_a);

    g.reconnect_ramp_scale = -1.0f;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mc_resolve(mc_snapshot(), g).left_a);

    g.reconnect_ramp_scale = std::nanf("");
    const MotorFrameCommand cmd = mc_resolve(mc_snapshot(), g);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.left_a);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.right_a);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_amps_offset);
    RUN_TEST(test_positive_amps);
    RUN_TEST(test_negative_regen);
    RUN_TEST(test_roundtrip);
    RUN_TEST(test_motor_speed_offset);
    RUN_TEST(test_encode_drive_control_frame);
    RUN_TEST(test_encode_regen_control_frame);
    RUN_TEST(test_encode_halted_zero_frame);
    RUN_TEST(test_ids);
    RUN_TEST(test_decode_cluster_command_bits);
    RUN_TEST(test_decode_cluster_command_regen_off);
    RUN_TEST(test_decode_controller_feedback);
    RUN_TEST(test_encode_vcu_cluster_status);
    RUN_TEST(test_encode_vcu_cluster_status_clears_invalid_throttle);
    RUN_TEST(test_sensor_telemetry_encoders);
    RUN_TEST(test_decode_cluster_bms_status_is_diagnostic);
    RUN_TEST(test_vehicle_speed_kph_to_raw_clamps_and_rounds);
    RUN_TEST(test_encode_vcu_vehicle_speed);
    RUN_TEST(test_mc_drive_commands_both_motors_forward);
    RUN_TEST(test_mc_reverse_commands_both_motors_backward);
    RUN_TEST(test_mc_regen_in_drive_targets_zero_rpm);
    RUN_TEST(test_mc_safety_halt_blocks_both);
    RUN_TEST(test_mc_non_propulsion_gear_blocks_both);
    RUN_TEST(test_mc_stale_snapshot_blocks_both);
    RUN_TEST(test_mc_reconnect_ramp_scales_both_sides_equally);
    RUN_TEST(test_mc_asymmetric_current_never_splits_direction);
    RUN_TEST(test_mc_non_finite_command_blocks_both_sides);
    RUN_TEST(test_mc_direction_not_armed_blocks_both);
    RUN_TEST(test_mc_invalid_throttle_signal_blocks_both);
    RUN_TEST(test_mc_core1_gates_block_both);
    RUN_TEST(test_mc_ramp_scale_is_clamped_and_nan_safe);
    return UNITY_END();
}
