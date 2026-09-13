#pragma once
#include <cstdint>

using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
constexpr esp_err_t ESP_FAIL = -1;

enum pcnt_unit_t {
    PCNT_UNIT_0,
    PCNT_UNIT_1,
    PCNT_UNIT_2,
    PCNT_UNIT_3,
};

enum pcnt_channel_t { PCNT_CHANNEL_0 };
enum pcnt_ctrl_mode_t { PCNT_MODE_KEEP };
enum pcnt_count_mode_t { PCNT_COUNT_DIS, PCNT_COUNT_INC };

constexpr int PCNT_PIN_NOT_USED = -1;

struct pcnt_config_t {
    int pulse_gpio_num;
    int ctrl_gpio_num;
    pcnt_ctrl_mode_t lctrl_mode;
    pcnt_ctrl_mode_t hctrl_mode;
    pcnt_count_mode_t pos_mode;
    pcnt_count_mode_t neg_mode;
    int16_t counter_h_lim;
    int16_t counter_l_lim;
    pcnt_unit_t unit;
    pcnt_channel_t channel;
};

esp_err_t pcnt_unit_config(const pcnt_config_t *config);
esp_err_t pcnt_set_filter_value(pcnt_unit_t unit, uint16_t value);
esp_err_t pcnt_filter_enable(pcnt_unit_t unit);
esp_err_t pcnt_counter_pause(pcnt_unit_t unit);
esp_err_t pcnt_counter_clear(pcnt_unit_t unit);
esp_err_t pcnt_counter_resume(pcnt_unit_t unit);
esp_err_t pcnt_get_counter_value(pcnt_unit_t unit, int16_t *count);
