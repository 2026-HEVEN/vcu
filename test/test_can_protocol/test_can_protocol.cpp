#include <unity.h>
#include "can_protocol.h"

void test_zero_amps_offset(void)   { TEST_ASSERT_EQUAL_UINT16(32000, torque_to_raw(0.0f)); }
void test_positive_amps(void)      { TEST_ASSERT_EQUAL_UINT16(32320, torque_to_raw(32.0f)); }
void test_negative_regen(void)     { TEST_ASSERT_EQUAL_UINT16(31680, torque_to_raw(-32.0f)); }
void test_roundtrip(void)          { TEST_ASSERT_FLOAT_WITHIN(0.01f, 17.5f, raw_to_torque(torque_to_raw(17.5f))); }
void test_ids(void) {
    TEST_ASSERT_EQUAL_HEX32(0x0C01EFD0, CAN_ID_TORQUE_L);
    TEST_ASSERT_EQUAL_HEX32(0x0C01F0D0, CAN_ID_TORQUE_R);
    TEST_ASSERT_EQUAL_HEX32(0x1803C0D0, CAN_ID_VCU_VEHICLE_SPEED);
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

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_amps_offset);
    RUN_TEST(test_positive_amps);
    RUN_TEST(test_negative_regen);
    RUN_TEST(test_roundtrip);
    RUN_TEST(test_ids);
    RUN_TEST(test_vehicle_speed_kph_to_raw_clamps_and_rounds);
    RUN_TEST(test_encode_vcu_vehicle_speed);
    return UNITY_END();
}
