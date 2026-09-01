// [FILL-IN] Edit this file. Implement the *_compute() function below.
#include "modules/throttle.h"
#include "modules/realcar_calibration.h"

// ══════ [변경 표시] 원본 대비 바뀐 부분 ═══════════════════════════════════
//  ① RAW_MIN/RAW_MAX 보정 상수 신규 추가 (아래)
//  ② compute() 로직 교체: "전체 ADC의 5% 데드존 선형" → "RAW_MIN~RAW_MAX 구간 매핑"
//     (원본: frac = raw/4095 → frac<0.05면 0 → (frac-0.05)/0.95*100)
// ═════════════════════════════════════════════════════════════════════════

// --- 보정값 (★실차에서 측정해 교체★) ------------------------------------
// 페달을 "안 밟은 상태"와 "끝까지 밟은 상태"의 raw_adc 를 읽어서 넣는다.
// (디버그 모니터나 Serial 로 in.raw_adc 를 보며 페달을 움직여 측정)
//   RAW_MIN : 0% 지점 = 쉼 상태 raw + 약간의 여유(노이즈/오프셋 흡수, 데드존)
//   RAW_MAX : 100% 지점 = 풀로 밟았을 때 raw
// 예) 홀 스로틀(0.5~4.5V)을 12bit ADC로 바로 읽으면 대략 RAW_MIN≈620, RAW_MAX≈3720.
static constexpr float RAW_MIN = realcar_cal::provisional::THROTTLE_RAW_MIN;
static constexpr float RAW_MAX = realcar_cal::provisional::THROTTLE_RAW_MAX;

Percent throttle_compute(const ThrottleInput &in) {
    // 데드존 아래(쉼 근처)는 0% — 음수/떨림 방지   ★변경: 기준을 frac<0.05 → raw<=RAW_MIN 으로
    if ((float)in.raw_adc <= RAW_MIN) return Percent(0.0f);

    // RAW_MIN..RAW_MAX 구간을 0..100% 로 선형 매핑   ★변경(핵심): 매핑식 교체
    float pct = (float)(in.raw_adc - RAW_MIN) / (RAW_MAX - RAW_MIN) * 100.0f;

    // Percent 타입이 상한 100% 로 자동 클램프 (RAW_MAX 초과 입력 대비)
    return Percent(pct);
}
