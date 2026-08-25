#include <unity.h>
#include "can_protocol.h"

void test_zero_amps_offset(void)   { TEST_ASSERT_EQUAL_UINT16(32000, torque_to_raw(0.0f)); }
void test_positive_amps(void)      { TEST_ASSERT_EQUAL_UINT16(32320, torque_to_raw(32.0f)); }
void test_negative_regen(void)     { TEST_ASSERT_EQUAL_UINT16(31680, torque_to_raw(-32.0f)); }
void test_roundtrip(void)          { TEST_ASSERT_FLOAT_WITHIN(0.01f, 17.5f, raw_to_torque(torque_to_raw(17.5f))); }
void test_ids(void) {
    TEST_ASSERT_EQUAL_HEX32(0x0C01EFD0, CAN_ID_TORQUE_L);
    TEST_ASSERT_EQUAL_HEX32(0x0C01F0D0, CAN_ID_TORQUE_R);
}
void test_feedback_ids(void) {
    TEST_ASSERT_EQUAL_HEX32(0x1801D0EF, CAN_ID_FB1_L);
    TEST_ASSERT_EQUAL_HEX32(0x1802D0EF, CAN_ID_FB2_L);
    TEST_ASSERT_EQUAL_HEX32(0x1801D0F0, CAN_ID_FB1_R);
    TEST_ASSERT_EQUAL_HEX32(0x1802D0F0, CAN_ID_FB2_R);
    // handshake request (FB1) and reply (TORQUE) IDs must be distinct per side
    TEST_ASSERT_NOT_EQUAL(CAN_ID_FB1_L, CAN_ID_FB1_R);
    TEST_ASSERT_NOT_EQUAL(CAN_ID_TORQUE_L, CAN_ID_TORQUE_R);
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
    RUN_TEST(test_feedback_ids);
    return UNITY_END();
}
