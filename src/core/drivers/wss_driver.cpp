// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "core/drivers/wss_driver.h"
#include <Arduino.h>
#include "driver/pcnt.h"

// [LOCKED] ESP-IDF legacy PCNT driver. Counts rising edges on the pulse input.
// ESP32에는 PCNT 유닛이 8개 있으므로 4채널은 유닛 0~3에 1:1로 배정한다.
// PCNT는 GPIO 매트릭스를 거치므로 input-only 핀(34/35/36/39)도 문제없다.
namespace {
    constexpr pcnt_unit_t UNITS[WHEEL_COUNT] = {
        PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3
    };
    constexpr pcnt_channel_t PCNT_CH = PCNT_CHANNEL_0;

    uint32_t last_ms_[WHEEL_COUNT]    = { 0, 0, 0, 0 };
    int16_t  last_count_[WHEEL_COUNT] = { 0, 0, 0, 0 };

    bool valid_ch(int ch) { return ch >= 0 && ch < WHEEL_COUNT; }
}

namespace wss_driver {

void begin(int ch, int gpio) {
    if (!valid_ch(ch)) return;
    const pcnt_unit_t unit = UNITS[ch];

    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = gpio;
    cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
    cfg.lctrl_mode     = PCNT_MODE_KEEP;
    cfg.hctrl_mode     = PCNT_MODE_KEEP;
    cfg.pos_mode       = PCNT_COUNT_INC;   // count on rising edge
    cfg.neg_mode       = PCNT_COUNT_DIS;   // ignore falling edge
    cfg.counter_h_lim  = 32767;
    cfg.counter_l_lim  = -1;
    cfg.unit           = unit;
    cfg.channel        = PCNT_CH;
    pcnt_unit_config(&cfg);

    // glitch filter: ~1 µs at 80 MHz APB = 80 cycles
    pcnt_set_filter_value(unit, 80);
    pcnt_filter_enable(unit);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
    last_ms_[ch] = millis();
    last_count_[ch] = 0;
}

WssReading read(int ch) {
    if (!valid_ch(ch)) return WssReading{ 0, 0 };
    int16_t count = 0;
    pcnt_get_counter_value(UNITS[ch], &count);
    uint32_t now = millis();
    WssReading r;
    r.pulse_delta = (uint32_t)(int16_t)(count - last_count_[ch]);
    r.dt_ms = now - last_ms_[ch];
    last_count_[ch] = count;
    last_ms_[ch] = now;
    return r;
}

} // namespace wss_driver
