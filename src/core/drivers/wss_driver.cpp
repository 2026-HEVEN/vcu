#include "core/drivers/wss_driver.h"
#include <Arduino.h>
#include "driver/pulse_cnt.h"

// [LOCKED] ESP-IDF PCNT v5 driver. Fill in unit config to match wiring.
namespace {
    pcnt_unit_handle_t  unit_ = nullptr;
    pcnt_channel_handle_t chan_ = nullptr;
    uint32_t last_ms_ = 0;
    int last_count_ = 0;
}

namespace wss_driver {

void begin(int gpio) {
    pcnt_unit_config_t ucfg = { .low_limit = -1, .high_limit = 32767 };
    pcnt_new_unit(&ucfg, &unit_);
    pcnt_chan_config_t ccfg = { .edge_gpio_num = gpio, .level_gpio_num = -1 };
    pcnt_new_channel(unit_, &ccfg, &chan_);
    pcnt_channel_set_edge_action(chan_,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    pcnt_glitch_filter_config_t fcfg = { .max_glitch_ns = 1000 };
    pcnt_unit_set_glitch_filter(unit_, &fcfg);
    pcnt_unit_enable(unit_);
    pcnt_unit_clear_count(unit_);
    pcnt_unit_start(unit_);
    last_ms_ = millis();
}

WssReading read() {
    int count = 0;
    pcnt_unit_get_count(unit_, &count);
    uint32_t now = millis();
    WssReading r;
    r.pulse_delta = (uint32_t)(count - last_count_);
    r.dt_ms = now - last_ms_;
    last_count_ = count;
    last_ms_ = now;
    return r;
}

} // namespace wss_driver
