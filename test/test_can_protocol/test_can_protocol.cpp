#include <unity.h>
#include <cmath>
#include "can_protocol.h"

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

// ── 고속 로깅 프레임 (Monolith 데이터로거용) ──────────────────────────────
// 4프레임 모두 Prio 7(0x1C...)로 보내 컨트롤러 Life Signal을 절대 지연시키지
// 않는다. Monolith가 수신 프레임 전체를 타임스탬프와 함께 raw로 SD에 남기므로
// life 바이트에 의존하지 않고, 대신 데이터 칸을 최대한 쓴다.

static int16_t i16(const uint8_t *d) { return (int16_t)(d[0] | (d[1] << 8)); }

void test_log_frame_ids_do_not_collide_with_existing(void) {
    // 0x1806/0x1807C0D0은 car_check(WSS/적용상태)가 이미 쓰고 있다.
    TEST_ASSERT_EQUAL_HEX32(0x1C01C0D0, CAN_ID_VCU_LOG_DRIVE);
    TEST_ASSERT_EQUAL_HEX32(0x1C02C0D0, CAN_ID_VCU_LOG_MOTOR);
    TEST_ASSERT_EQUAL_HEX32(0x1C03C0D0, CAN_ID_VCU_LOG_TV_YAW);
    TEST_ASSERT_EQUAL_HEX32(0x1C04C0D0, CAN_ID_VCU_LOG_TV_LOAD);
    TEST_ASSERT_NOT_EQUAL(CAN_ID_VCU_WHEEL_SPEEDS, CAN_ID_VCU_LOG_DRIVE);
    TEST_ASSERT_NOT_EQUAL(CAN_ID_VCU_CONTROL_STATUS, CAN_ID_VCU_LOG_MOTOR);
}

void test_encode_log_drive(void) {
    uint8_t d[8];
    encode_vcu_log_drive(120.5f, -40.25f, 118.0f, -39.0f, d);
    TEST_ASSERT_EQUAL_INT16(1205, i16(d + 0));   // 0.1 A/bit
    TEST_ASSERT_EQUAL_INT16(-403, i16(d + 2));   // 반올림
    TEST_ASSERT_EQUAL_INT16(1180, i16(d + 4));
    TEST_ASSERT_EQUAL_INT16(-390, i16(d + 6));
}

void test_encode_log_motor(void) {
    uint8_t d[8];
    encode_vcu_log_motor(2500, -1200, 33.3f, -8.1f, d);
    TEST_ASSERT_EQUAL_INT16(2500, i16(d + 0));   // 1 rpm/bit
    TEST_ASSERT_EQUAL_INT16(-1200, i16(d + 2));
    TEST_ASSERT_EQUAL_INT16(333, i16(d + 4));    // 0.1 A/bit
    TEST_ASSERT_EQUAL_INT16(-81, i16(d + 6));
}

void test_encode_log_tv_yaw(void) {
    uint8_t d[8];
    encode_vcu_log_tv_yaw(12.34f, -56.78f, 200.0f, 210.0f, d);
    TEST_ASSERT_EQUAL_INT16(1234, i16(d + 0));   // 0.01 deg/s
    TEST_ASSERT_EQUAL_INT16(-5678, i16(d + 2));  // 0.01 N·m
    TEST_ASSERT_EQUAL_INT16(2000, i16(d + 4));   // 0.1 A
    TEST_ASSERT_EQUAL_INT16(2100, i16(d + 6));
}

void test_encode_log_tv_load(void) {
    uint8_t d[8];
    // max_L/max_R은 진단용 한계라 4 A/bit uint8로 충분하다 (게이트 비트 자리 확보)
    encode_vcu_log_tv_load(598.0f, 720.0f, 303.0f, 288.0f, 0x2Bu, 7u, d);
    TEST_ASSERT_EQUAL_INT16(598, i16(d + 0));    // 1 N/bit
    TEST_ASSERT_EQUAL_INT16(720, i16(d + 2));
    TEST_ASSERT_EQUAL_UINT8(76, d[4]);           // 303/4 = 75.75 → 반올림 76
    TEST_ASSERT_EQUAL_UINT8(72, d[5]);           // 288/4 = 72
    TEST_ASSERT_EQUAL_UINT8(0x2B, d[6]);         // 게이트 비트필드
    TEST_ASSERT_EQUAL_UINT8(7, d[7]);            // life
}

void test_log_encoders_are_nan_safe(void) {
    // TV 내부 신호는 NaN이 될 수 있다. telemetry_to_i16의 기존 클램프는
    // NaN을 걸러내지 못해 정의되지 않은 캐스트로 빠진다. 0으로 떨어뜨린다.
    const float nan_value = NAN;
    uint8_t d[8];
    encode_vcu_log_tv_yaw(nan_value, nan_value, nan_value, nan_value, d);
    TEST_ASSERT_EQUAL_INT16(0, i16(d + 0));
    TEST_ASSERT_EQUAL_INT16(0, i16(d + 2));
}

void test_log_drive_clamps_out_of_range(void) {
    uint8_t d[8];
    encode_vcu_log_drive(9999.0f, -9999.0f, 0.0f, 0.0f, d);
    TEST_ASSERT_EQUAL_INT16(32767, i16(d + 0));
    TEST_ASSERT_EQUAL_INT16(-32768, i16(d + 2));
}

void test_encode_log_clamp(void) {
    uint8_t d[8];
    // 카운터는 누적값이다. 1Hz로 보내면 로그에서 증분이 그 1초 구간의
    // 포화 횟수가 되어 시간 정보가 복원된다.
    encode_vcu_log_clamp(37u, 2u, 812.4f, -540.0f, d);
    TEST_ASSERT_EQUAL_UINT16(37, (uint16_t)(d[0] | (d[1] << 8)));
    TEST_ASSERT_EQUAL_UINT16(2,  (uint16_t)(d[2] | (d[3] << 8)));
    TEST_ASSERT_EQUAL_INT16(8124, i16(d + 4));   // 0.1 A/bit
    TEST_ASSERT_EQUAL_INT16(-5400, i16(d + 6));
}

void test_log_clamp_saturates_counters(void) {
    // uint32 카운터를 uint16으로 좁히므로 65535에서 멈춰야 한다.
    // 감싸돌면 증분이 음수가 되어 로그 분석이 깨진다.
    uint8_t d[8];
    encode_vcu_log_clamp(70000u, 99999u, 0.0f, 0.0f, d);
    TEST_ASSERT_EQUAL_UINT16(65535, (uint16_t)(d[0] | (d[1] << 8)));
    TEST_ASSERT_EQUAL_UINT16(65535, (uint16_t)(d[2] | (d[3] << 8)));
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
    RUN_TEST(test_log_frame_ids_do_not_collide_with_existing);
    RUN_TEST(test_encode_log_drive);
    RUN_TEST(test_encode_log_motor);
    RUN_TEST(test_encode_log_tv_yaw);
    RUN_TEST(test_encode_log_tv_load);
    RUN_TEST(test_log_encoders_are_nan_safe);
    RUN_TEST(test_log_drive_clamps_out_of_range);
    RUN_TEST(test_encode_log_clamp);
    RUN_TEST(test_log_clamp_saturates_counters);
    return UNITY_END();
}
