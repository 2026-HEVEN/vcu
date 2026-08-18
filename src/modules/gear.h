#pragma once
#include <cstdint>
// [FILL-IN] D/R/N 기어 셀렉터. 저항 사다리 ADC → 기어 상태.
//   D27(아날로그 ADC)에 물린 저항 사다리가 위치마다 다른 전압을 낸다.
//   각 위치의 공칭 ADC ± tolerance 밴드로 판별. 어느 밴드에도 안 맞으면(플로팅·고장)
//   안전상 Neutral 로 떨어진다(= 구동 금지).

enum class Gear { Neutral, Drive, Reverse };   // 안전 기본값 = Neutral

struct GearRaw   { uint16_t counts; };          // ADC 0..4095 (D27 저항 사다리)
struct GearCalib {
    uint16_t reverse_adc;   // R 위치 공칭 ADC (실측)
    uint16_t neutral_adc;   // N 위치 공칭 ADC
    uint16_t drive_adc;     // D 위치 공칭 ADC
    uint16_t tolerance;     // ± 밴드 폭. 이 범위 밖이면 판별 실패 → Neutral
};

Gear gear_compute(const GearRaw &raw, const GearCalib &c);
