#include <unity.h>
#include "../../src/core/board_pins.h"

void setUp() {}
void tearDown() {}

void test_pcb_v3_vcu_pin_contract() {
    TEST_ASSERT_EQUAL_INT(23, board_pins::CAN_RX);
    TEST_ASSERT_EQUAL_INT(22, board_pins::CAN_TX);
    TEST_ASSERT_EQUAL_INT(36, board_pins::STEERING_ADC);
    TEST_ASSERT_EQUAL_INT(39, board_pins::BRAKE_PRESSURE_ADC);
    TEST_ASSERT_EQUAL_INT(34, board_pins::THROTTLE_ADC);
    TEST_ASSERT_EQUAL_INT(35, board_pins::BRAKE_ONOFF_ADC);
    TEST_ASSERT_EQUAL_INT(32, board_pins::GEAR_ADC);
    TEST_ASSERT_EQUAL_INT(33, board_pins::LV_VOLTAGE_ADC);
    TEST_ASSERT_EQUAL_INT(18, board_pins::WSS_FL);
    TEST_ASSERT_EQUAL_INT(17, board_pins::WSS_FR);
    TEST_ASSERT_EQUAL_INT(16, board_pins::WSS_RL);
    TEST_ASSERT_EQUAL_INT(4, board_pins::WSS_RR);
    TEST_ASSERT_EQUAL_INT(21, board_pins::IMU_RX);
    TEST_ASSERT_EQUAL_INT(19, board_pins::IMU_TX);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_pcb_v3_vcu_pin_contract);
    return UNITY_END();
}
