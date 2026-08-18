// [FILL-IN] Edit this file. 저항 사다리 ADC → D/R/N 기어.
#include "modules/gear.h"

namespace {
uint16_t adiff(uint16_t a, uint16_t b) { return a > b ? (uint16_t)(a - b) : (uint16_t)(b - a); }
}

// 세 위치 공칭 ADC 중 raw에 가장 가까운 것을 고른다. 단 tolerance 안이어야 채택.
// 어느 위치와도 tolerance 내로 맞지 않으면(플로팅·단선·중간위치) 안전상 Neutral.
Gear gear_compute(const GearRaw &raw, const GearCalib &c) {
    Gear     best  = Gear::Neutral;   // fail-safe 기본값
    uint16_t bestd = c.tolerance;     // 이 값 이하로 가까워야 채택

    uint16_t dR = adiff(raw.counts, c.reverse_adc);
    if (dR <= bestd) { bestd = dR; best = Gear::Reverse; }
    uint16_t dN = adiff(raw.counts, c.neutral_adc);
    if (dN <= bestd) { bestd = dN; best = Gear::Neutral; }
    uint16_t dD = adiff(raw.counts, c.drive_adc);
    if (dD <= bestd) { bestd = dD; best = Gear::Drive; }

    return best;   // 가장 가까운(≤tolerance) 위치, 없으면 Neutral
}
