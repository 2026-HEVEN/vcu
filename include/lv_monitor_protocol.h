#pragma once
#include <cstdint>

// PCB V3: +12V_FUSED -> 3.3k / 1k divider -> GPIO33.
// Extended CAN, DLC 8, 100 ms, diagnostic only (never a drive gate).
// [0:1] LV centivolts u16 LE; [2:3] ADC millivolts u16 LE;
// [4] bit0 sample usable (not proof of connected wiring); [5] sequence;
// [6:7] reserved zero. Reject clipped ADC readings. Zero volts is measurable.
namespace lv_monitor {
constexpr uint32_t CAN_ID = 0x1C09C0D0;
constexpr uint32_t FRESH_MS = 500;
constexpr uint32_t ADC_MAX_MV = 3100;
struct Sample {
    uint16_t centivolts;
    uint16_t adc_mv;
    bool valid;
    uint8_t sequence;
    Sample(uint16_t cv = 0, uint16_t mv = 0, bool usable = false, uint8_t seq = 0)
        : centivolts(cv), adc_mv(mv), valid(usable), sequence(seq) {}
};
inline Sample from_adc(uint32_t mv, uint8_t sequence) {
    return {static_cast<uint16_t>(mv < ADC_MAX_MV ? (mv * 43U + 50U) / 100U : 0),
            static_cast<uint16_t>(mv > 65535U ? 65535U : mv),
            mv < ADC_MAX_MV, sequence};
}
inline void encode(const Sample &s, uint8_t out[8]) {
    out[0] = s.centivolts; out[1] = s.centivolts >> 8;
    out[2] = s.adc_mv; out[3] = s.adc_mv >> 8;
    out[4] = s.valid ? 1 : 0; out[5] = s.sequence;
    out[6] = out[7] = 0;
}
inline Sample decode(const uint8_t d[8]) {
    return {static_cast<uint16_t>(d[0] | (uint16_t(d[1]) << 8)),
            static_cast<uint16_t>(d[2] | (uint16_t(d[3]) << 8)),
            (d[4] & 1) != 0, d[5]};
}
inline bool fresh(bool seen, uint32_t last_ms, uint32_t now_ms, const Sample &s) {
    return seen && s.valid && uint32_t(now_ms - last_ms) <= FRESH_MS;
}
}
