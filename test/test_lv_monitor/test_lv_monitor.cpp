#include <unity.h>
#include "lv_monitor_protocol.h"
void test_voltage_and_wire() {
    auto s = lv_monitor::from_adc(2791, 42);
    TEST_ASSERT_TRUE(s.valid);
    TEST_ASSERT_EQUAL_UINT16(1200, s.centivolts);
    uint8_t d[8]; lv_monitor::encode(s, d);
    TEST_ASSERT_EQUAL_HEX8(0xB0, d[0]);
    TEST_ASSERT_EQUAL_HEX8(0x04, d[1]);
    auto r = lv_monitor::decode(d);
    TEST_ASSERT_EQUAL_UINT16(2791, r.adc_mv);
    TEST_ASSERT_EQUAL_UINT16(1200, r.centivolts);
    TEST_ASSERT_EQUAL_UINT8(42, r.sequence);
    TEST_ASSERT_TRUE(r.valid);
    TEST_ASSERT_EQUAL_UINT8(0, d[6] | d[7]);
    TEST_ASSERT_EQUAL_UINT16(1000, lv_monitor::from_adc(2326, 0).centivolts);
    TEST_ASSERT_TRUE(lv_monitor::from_adc(0, 0).valid);
    TEST_ASSERT_FALSE(lv_monitor::from_adc(3100, 0).valid);
    TEST_ASSERT_FALSE(lv_monitor::from_adc(0xFFFFFFFFU, 0).valid);
}
void test_freshness() {
    auto s = lv_monitor::from_adc(2791, 0);
    TEST_ASSERT_FALSE(lv_monitor::fresh(false, 0, 0, s));
    TEST_ASSERT_TRUE(lv_monitor::fresh(true, 100, 600, s));
    TEST_ASSERT_FALSE(lv_monitor::fresh(true, 100, 601, s));
    TEST_ASSERT_TRUE(lv_monitor::fresh(true, 0xFFFFFFF0U, 20, s));
    s.valid = false;
    TEST_ASSERT_FALSE(lv_monitor::fresh(true, 100, 100, s));
}
void setUp() {}
void tearDown() {}
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_voltage_and_wire);
    RUN_TEST(test_freshness);
    return UNITY_END();
}

