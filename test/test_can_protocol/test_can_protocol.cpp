#include <unity.h>
#include "can_protocol.h"

void test_zero_amps_offset(void)   { TEST_ASSERT_EQUAL_UINT16(32000, torque_to_raw(0.0f)); }
void test_positive_amps(void)      { TEST_ASSERT_EQUAL_UINT16(32320, torque_to_raw(32.0f)); }
void test_negative_regen(void)     { TEST_ASSERT_EQUAL_UINT16(31680, torque_to_raw(-32.0f)); }
void test_roundtrip(void)          { TEST_ASSERT_FLOAT_WITHIN(0.01f, 17.5f, raw_to_torque(torque_to_raw(17.5f))); }
void test_ids(void) {
    TEST_ASSERT_EQUAL_HEX32(0x0C01EFD0, CAN_ID_TORQUE_L);
    TEST_ASSERT_EQUAL_HEX32(0x0C01F0D0, CAN_ID_TORQUE_R);
    TEST_ASSERT_EQUAL_HEX32(0x1802C0D0, CAN_ID_VCU_WHEEL_SPEEDS);
}

void test_wheel_rpm_to_raw_clamps_and_rounds(void) {
    TEST_ASSERT_EQUAL_UINT16(0, wheel_rpm_to_raw(-1.0f));
    TEST_ASSERT_EQUAL_UINT16(123, wheel_rpm_to_raw(123.4f));
    TEST_ASSERT_EQUAL_UINT16(124, wheel_rpm_to_raw(123.5f));
    TEST_ASSERT_EQUAL_UINT16(65535, wheel_rpm_to_raw(70000.0f));
}

void test_encode_vcu_wheel_speeds(void) {
    uint8_t out[8];
    encode_vcu_wheel_speeds(100.0f, 200.0f, 300.0f, 400.0f, out);
    TEST_ASSERT_EQUAL_UINT8(0x64, out[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[1]);
    TEST_ASSERT_EQUAL_UINT8(0xC8, out[2]);
    TEST_ASSERT_EQUAL_UINT8(0x00, out[3]);
    TEST_ASSERT_EQUAL_UINT8(0x2C, out[4]);
    TEST_ASSERT_EQUAL_UINT8(0x01, out[5]);
    TEST_ASSERT_EQUAL_UINT8(0x90, out[6]);
    TEST_ASSERT_EQUAL_UINT8(0x01, out[7]);
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
    RUN_TEST(test_wheel_rpm_to_raw_clamps_and_rounds);
    RUN_TEST(test_encode_vcu_wheel_speeds);
    return UNITY_END();
}
