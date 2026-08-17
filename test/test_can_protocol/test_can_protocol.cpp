#include <unity.h>
#include "can_protocol.h"

void test_zero_amps_offset(void)   { TEST_ASSERT_EQUAL_UINT16(32000, torque_to_raw(0.0f)); }
void test_positive_amps(void)      { TEST_ASSERT_EQUAL_UINT16(32320, torque_to_raw(32.0f)); }
void test_negative_regen(void)     { TEST_ASSERT_EQUAL_UINT16(31680, torque_to_raw(-32.0f)); }
void test_roundtrip(void)          { TEST_ASSERT_FLOAT_WITHIN(0.01f, 17.5f, raw_to_torque(torque_to_raw(17.5f))); }
void test_ids(void) {
    TEST_ASSERT_EQUAL_HEX32(0x0C01EFD0, CAN_ID_TORQUE_L);
    TEST_ASSERT_EQUAL_HEX32(0x0C01F0D0, CAN_ID_TORQUE_R);
    TEST_ASSERT_EQUAL_HEX32(0x1801D0C0, CAN_ID_CLUSTER_CMD);
    TEST_ASSERT_EQUAL_HEX32(0x1803C0D0, CAN_ID_VCU_VEHICLE_SPEED);
    TEST_ASSERT_EQUAL_HEX32(0x1804C0D0, CAN_ID_VCU_STEERING);
    TEST_ASSERT_EQUAL_HEX32(0x1805C0D0, CAN_ID_VCU_IMU);
}

void test_decode_cluster_command_bits(void) {
    uint8_t data[8] = {};
    data[1] = 0x0B; // TC + Regen Auto + Debug, bit2 reserved clear
    data[2] = 0x01; // Paddock

    ClusterCommandRequest cmd = decode_cluster_command(data);
    TEST_ASSERT_TRUE(cmd.tc_enabled);
    TEST_ASSERT_TRUE(cmd.regen_auto_enabled);
    TEST_ASSERT_TRUE(cmd.debug_enabled);
    TEST_ASSERT_TRUE(cmd.paddock_request);
}

void test_decode_cluster_command_regen_off(void) {
    uint8_t data[8] = {};
    data[1] = 0x04; // reserved bit must not imply regen auto
    ClusterCommandRequest cmd = decode_cluster_command(data);
    TEST_ASSERT_FALSE(cmd.tc_enabled);
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

void test_telemetry_to_i16_clamps_and_rounds(void) {
    TEST_ASSERT_EQUAL_INT16(1235, telemetry_to_i16(1.2345f, 1000.0f));
    TEST_ASSERT_EQUAL_INT16(-1235, telemetry_to_i16(-1.2345f, 1000.0f));
    TEST_ASSERT_EQUAL_INT16(32767, telemetry_to_i16(400.0f, 100.0f));
    TEST_ASSERT_EQUAL_INT16(-32768, telemetry_to_i16(-400.0f, 100.0f));
}

void test_encode_vcu_steering(void) {
    uint8_t out[8];
    encode_vcu_steering(-0.375f, out);
    TEST_ASSERT_EQUAL_UINT8(0x89, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0xFE, out[1]);
    for (int i = 2; i < 8; ++i) TEST_ASSERT_EQUAL_UINT8(0, out[i]);
}

void test_encode_vcu_imu(void) {
    uint8_t out[8];
    encode_vcu_imu(12.34f, -0.56f, 1.25f, out);
    TEST_ASSERT_EQUAL_UINT8(0xD2, out[0]); // 1234
    TEST_ASSERT_EQUAL_UINT8(0x04, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0xC8, out[2]); // -56
    TEST_ASSERT_EQUAL_UINT8(0xFF, out[3]);
    TEST_ASSERT_EQUAL_UINT8(0x7D, out[4]); // 125
    TEST_ASSERT_EQUAL_UINT8(0x00, out[5]);
    TEST_ASSERT_EQUAL_UINT8(0, out[6]);
    TEST_ASSERT_EQUAL_UINT8(0, out[7]);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_amps_offset);
    RUN_TEST(test_positive_amps);
    RUN_TEST(test_negative_regen);
    RUN_TEST(test_roundtrip);
    RUN_TEST(test_ids);
    RUN_TEST(test_decode_cluster_command_bits);
    RUN_TEST(test_decode_cluster_command_regen_off);
    RUN_TEST(test_vehicle_speed_kph_to_raw_clamps_and_rounds);
    RUN_TEST(test_encode_vcu_vehicle_speed);
    RUN_TEST(test_telemetry_to_i16_clamps_and_rounds);
    RUN_TEST(test_encode_vcu_steering);
    RUN_TEST(test_encode_vcu_imu);
    return UNITY_END();
}
