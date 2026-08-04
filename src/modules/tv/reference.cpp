// [FILL-IN] Stage 1/5 — 레퍼런스 모델   담당: ______
// NOTE(AI 구현): 이 함수는 Claude와 함께 작성함 — 담당자 검토 후 이 줄 삭제.
#include "modules/tv/reference.h"
#include <cmath>

namespace {
constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;

// ESP32(xtensa) 툴체인 libstdc++에 std::clamp(C++17)가 없어 직접 구현.
float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
}

// ── 이 함수가 하는 일 ─────────────────────────────────────────────
//   운전자의 "조향 의도"를, 차가 이상적으로 내야 할 "목표 yaw rate"로 바꾼다.
//   다음 단계(yaw_control)가 이 목표와 IMU 실측 yaw rate를 비교해 오차를 만든다.
float tv_reference_compute(Unit steering, float speed_mps, const TVParams &p) {
    // 저속 컷오프: 이 밑에서는 정상원선회 모델 자체가 신뢰할 수 없고,
    // 분모(wheelbase + Kus*V^2)도 0에 가까워질 수 있어 나눗셈이 불안정해진다.
    if (std::fabs(speed_mps) < p.min_speed_mps) {
        return 0.0f;
    }

    // steering: Unit(-1..+1) → 실제 타이어 조향각(rad). 좌회전이 +가 되도록
    // p.max_steer_rad 부호를 IMU yaw rate(좌회전 +)와 통일해서 맞춘 상태라고 가정.
    float delta_rad = (float)steering * p.max_steer_rad;

    float denom = p.wheelbase_m + p.understeer_grad * speed_mps * speed_mps;
    if (denom < 1e-4f) {
        return 0.0f;
    }

    // 정상원선회 바이시클 모델: r = V * delta / (L + Kus * V^2)   [rad/s]
    float target_rad_s = (speed_mps * delta_rad) / denom;
    float target_deg_s = target_rad_s * kRadToDeg;

    // 상한 clamp — 저속/큰 조향에서 목표가 비현실적으로 커지는 것을 방지.
    return clampf(target_deg_s, -p.desired_yaw_max, p.desired_yaw_max);
}
