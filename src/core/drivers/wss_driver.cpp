#include "core/drivers/wss_driver.h"
#include <Arduino.h>
#include "driver/pcnt.h"

// [LOCKED] ESP-IDF legacy PCNT driver. Counts rising edges on the pulse input.
namespace {
    constexpr pcnt_unit_t  PCNT_UNIT = PCNT_UNIT_0;
    constexpr pcnt_channel_t PCNT_CH = PCNT_CHANNEL_0;
    uint32_t last_ms_ = 0;
    int16_t  last_count_ = 0;
}

namespace wss_driver {

void begin(int gpio) {
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = gpio;
    cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
    cfg.lctrl_mode     = PCNT_MODE_KEEP;
    cfg.hctrl_mode     = PCNT_MODE_KEEP;
    cfg.pos_mode       = PCNT_COUNT_INC;   // count on rising edge
    cfg.neg_mode       = PCNT_COUNT_DIS;   // ignore falling edge
    cfg.counter_h_lim  = 32767;
    cfg.counter_l_lim  = -1;
    cfg.unit           = PCNT_UNIT;
    cfg.channel        = PCNT_CH;
    pcnt_unit_config(&cfg);

    // glitch filter: ~1 µs at 80 MHz APB = 80 cycles
    pcnt_set_filter_value(PCNT_UNIT, 80);
    pcnt_filter_enable(PCNT_UNIT);

    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
    last_ms_ = millis();
}

WssReading read() {
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT, &count);
    uint32_t now = millis();
    WssReading r;
    r.pulse_delta = (uint32_t)(count - last_count_);
    r.dt_ms = now - last_ms_;
    last_count_ = count;
    last_ms_ = now;
    return r;
}

} // namespace wss_driver
