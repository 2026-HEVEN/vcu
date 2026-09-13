#pragma once
#include <cstdint>
#include "types.h"
// [FILL-IN] Raw sensor counts -> normalized steering. Calibration owned by team.
// 현재 실차는 D25의 12-bit ADC 슬라이드 포텐셔미터를 사용하고 드라이버가
// 기존 인터페이스와 맞도록 14-bit 범위로 스케일한다.

struct SteerRaw   { uint16_t counts; };                          // 12-bit ADC -> 0..16380
struct SteerCalib { uint16_t center_counts; float counts_per_unit; bool invert; };

Unit steering_compute(const SteerRaw &raw, const SteerCalib &c);

// 조향 샘플을 믿을 수 있는가. 센서 미설치(ADC 핀이 떠 있음), ADC 레일 근처
// (단선·단락), 계산 결과가 유한하지 않으면 false. 계기판 valid와 TV 게이트가
// 같은 판단을 쓰도록 여기 한 곳에 둔다.
bool steering_sample_valid(const SteerRaw &raw, float steering_unit,
                           bool sensor_installed);
