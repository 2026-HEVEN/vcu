// [FILL-IN] Stage 4/5 — 트랙션 한계(마찰원)   담당: ______
// NOTE(AI 구현): 이 함수는 Claude와 함께 작성함 — 담당자 검토 후 이 줄 삭제.
// 원래 계획은 휠스피드 기반 슬립 제한이었지만, 이 헤더는 휠스피드를 안 받는다
// (휠스피드는 잠긴 core/app_wiring.cpp의 TVInput 확장이 필요 — 별도 논의 필요).
// 그래서 헤더가 이미 지원하는 Fz×μ 마찰원 모델로 대신 구현했다.
#include "modules/tv/traction.h"
#include <cmath>

// ── 이 함수가 하는 일 ─────────────────────────────────────────────
//   각 바퀴가 미끄러지지 않고 낼 수 있는 "최대 전류"를 Fz와 노면 μ로 계산한다.
//   allocation stage가 이 상한을 넘지 않게 좌우 전류를 배분한다(휠스핀/그립상실 방지).
//   단위는 A로 통일 — allocation의 total_torque/출력이 이미 A이기 때문.
//
// ── Fy 근사에 대한 주의 ────────────────────────────────────────────
//   이 바퀴가 실제로 쓰고 있는 횡력(Fy)을 직접 아는 방법이 없다(횡력 센서 없음,
//   Cf/Cr 미식별). 그래서 "구동축 전체가 부담하는 횡력 = 총 횡력 × 정적 하중배분
//   비율(weight_dist_r)"이라는 보수적 근사만 쓴다. 실측·식별이 끝나기 전까지는
//   이 근사가 실제보다 낙관적일 수 있으니 p.mu를 낮게(보수적으로) 잡아 상쇄한다.
namespace {
float wheel_current_limit(float fz_w, float fz_axle, float fy_axle, const TVParams &p) {
    if (fz_w <= 0.0f || fz_axle <= 0.0f) {
        return 0.0f;   // 하중이 없거나(들림) 축 전체가 0이면 낼 수 있는 토크도 0.
    }

    // 이 바퀴가 축 전체 횡력 중 자기 하중 비율만큼 부담한다고 가정.
    float fy_w = fy_axle * (fz_w / fz_axle);

    float grip = p.mu * fz_w;              // 이 바퀴가 낼 수 있는 최대 마찰력(원의 반지름)
    float grip_sq = grip * grip;
    float fy_sq = fy_w * fy_w;
    // 마찰원: Fx_max = sqrt(max(0, grip^2 - Fy^2)) — 횡을 많이 쓸수록 종 여유가 준다.
    float fx_max = (grip_sq > fy_sq) ? std::sqrt(grip_sq - fy_sq) : 0.0f;

    float denom = p.kt_nm_per_a * p.gear_ratio;
    if (denom < 1e-4f) {
        return 0.0f;
    }
    float torque_max_nm = fx_max * p.tire_radius_m;
    return torque_max_nm / denom;          // N·m → A
}
}

MaxTorque tv_traction_compute(WheelLoads fz, float ay, const TVParams &p) {
    float fz_axle = fz.fz_L + fz.fz_R;
    float total_fy = p.mass_kg * std::fabs(ay) * p.weight_dist_r;

    MaxTorque out;
    out.max_L = wheel_current_limit(fz.fz_L, fz_axle, total_fy, p);
    out.max_R = wheel_current_limit(fz.fz_R, fz_axle, total_fy, p);
    return out;
}
