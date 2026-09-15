#include "modules/tv/load.h"
#include <cmath>

namespace {
constexpr float G_MPS2 = 9.80665f;
float nonnegative(float value) { return value > 0.0f ? value : 0.0f; }
}

WheelLoads tv_load_compute(GForce ax_q, GForce ay_q, const TVParams &p) {
    const float ax_g = (float)ax_q;
    const float ay_g = (float)ay_q;
    if (!std::isfinite(ax_g) || !std::isfinite(ay_g) || p.mass_kg <= 0.0f ||
        p.wheelbase_m <= 0.0f || p.track_m <= 0.0f)
        return {Newton{}, Newton{}};

    const float rear_static = p.mass_kg * G_MPS2 * p.weight_dist_r;
    const float rear_long_transfer =
        p.mass_kg * (ax_g * G_MPS2) * p.cg_height_m / p.wheelbase_m;
    const float rear_total = nonnegative(rear_static + rear_long_transfer);

    // dFz_lr은 "바퀴당" 하중 편차다(좌우 차이가 아니다). 롤 모멘트 평형으로:
    //   후축이 받는 롤 모멘트 몫  =  lltd_r · m · ay · h
    //   좌우 수직하중이 이를 버틴다:  (F_R − F_L) · (t/2) = lltd_r · m · ay · h
    //   →  F_R − F_L   = 2 · lltd_r · m · ay · h / t
    //   →  바퀴당 편차  =     lltd_r · m · ay · h / t   ← 아래 식이 계산하는 값
    // 따라서 rear_total만 반으로 나누고 dFz_lr은 그대로 더하고 뺀다. 이전 판은
    // dFz_lr을 좌우 차이로 간주해 0.5를 한 번 더 곱했고, 그 결과 횡하중 이동이
    // 절반으로 축소됐다. Stage 4의 마찰원 한계가 이 Fz를 쓰므로 바깥 바퀴
    // 한계를 과소, 안쪽 바퀴 한계를 과대평가하게 된다.
    const float dFz_lr = p.lltd_r * p.mass_kg * (ay_g * G_MPS2) *
                         p.cg_height_m / p.track_m;
    const float half_axle = 0.5f * rear_total;
    return {
        Newton{nonnegative(half_axle - dFz_lr)},
        Newton{nonnegative(half_axle + dFz_lr)},
    };
}
