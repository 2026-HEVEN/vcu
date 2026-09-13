#include <unity.h>
#include "driver/pcnt.h"

namespace {
uint32_t mock_ms;
int16_t mock_count[4];
bool mock_read_fails;
int16_t configured_high_limit;
}

uint32_t millis() { return mock_ms; }

esp_err_t pcnt_unit_config(const pcnt_config_t *config) {
    configured_high_limit = config->counter_h_lim;
    return ESP_OK;
}
esp_err_t pcnt_set_filter_value(pcnt_unit_t, uint16_t) { return ESP_OK; }
esp_err_t pcnt_filter_enable(pcnt_unit_t) { return ESP_OK; }
esp_err_t pcnt_counter_pause(pcnt_unit_t) { return ESP_OK; }
esp_err_t pcnt_counter_clear(pcnt_unit_t unit) {
    mock_count[unit] = 0;
    return ESP_OK;
}
esp_err_t pcnt_counter_resume(pcnt_unit_t) { return ESP_OK; }
esp_err_t pcnt_get_counter_value(pcnt_unit_t unit, int16_t *count) {
    if (mock_read_fails) return ESP_FAIL;
    *count = mock_count[unit];
    return ESP_OK;
}

#include "../../src/core/drivers/wss_driver.cpp"

void setUp() {
    mock_ms = 0;
    mock_read_fails = false;
    configured_high_limit = 0;
    for (int ch = 0; ch < 4; ++ch) mock_count[ch] = 0;
    wss_driver::begin(0, 36);
}

void tearDown() {}

void test_configures_the_same_limit_used_for_rollover() {
    TEST_ASSERT_EQUAL_INT16(32767, configured_high_limit);
}

void test_normal_delta() {
    mock_count[0] = 100;
    mock_ms = 10;
    TEST_ASSERT_EQUAL_UINT32(100, wss_driver::read(0).pulse_delta);

    mock_count[0] = 104;
    mock_ms = 20;
    const WssReading reading = wss_driver::read(0);
    TEST_ASSERT_EQUAL_UINT32(4, reading.pulse_delta);
    TEST_ASSERT_EQUAL_UINT32(10, reading.dt_ms);
}

void test_rollover_delta_from_32765_to_2_is_four() {
    mock_count[0] = 32765;
    mock_ms = 10;
    wss_driver::read(0);

    mock_count[0] = 2;
    mock_ms = 20;
    TEST_ASSERT_EQUAL_UINT32(4, wss_driver::read(0).pulse_delta);
}

void test_rollover_delta_from_32766_to_0_is_one() {
    mock_count[0] = 32766;
    mock_ms = 10;
    wss_driver::read(0);

    mock_count[0] = 0;
    mock_ms = 20;
    TEST_ASSERT_EQUAL_UINT32(1, wss_driver::read(0).pulse_delta);
}

void test_stopped_wheel_has_zero_delta() {
    mock_ms = 10;
    const WssReading reading = wss_driver::read(0);
    TEST_ASSERT_EQUAL_UINT32(0, reading.pulse_delta);
    TEST_ASSERT_EQUAL_UINT32(10, reading.dt_ms);
}

void test_failed_read_preserves_last_successful_baseline() {
    mock_count[0] = 100;
    mock_ms = 10;
    wss_driver::read(0);

    mock_read_fails = true;
    mock_ms = 20;
    const WssReading failed = wss_driver::read(0);
    TEST_ASSERT_EQUAL_UINT32(0, failed.pulse_delta);
    TEST_ASSERT_EQUAL_UINT32(0, failed.dt_ms);
    TEST_ASSERT_FALSE(wss_driver::last_read_ok(0));

    mock_read_fails = false;
    mock_count[0] = 104;
    mock_ms = 30;
    const WssReading recovered = wss_driver::read(0);
    TEST_ASSERT_EQUAL_UINT32(4, recovered.pulse_delta);
    TEST_ASSERT_EQUAL_UINT32(20, recovered.dt_ms);
    TEST_ASSERT_TRUE(wss_driver::last_read_ok(0));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_configures_the_same_limit_used_for_rollover);
    RUN_TEST(test_normal_delta);
    RUN_TEST(test_rollover_delta_from_32765_to_2_is_four);
    RUN_TEST(test_rollover_delta_from_32766_to_0_is_one);
    RUN_TEST(test_stopped_wheel_has_zero_delta);
    RUN_TEST(test_failed_read_preserves_last_successful_baseline);
    return UNITY_END();
}
