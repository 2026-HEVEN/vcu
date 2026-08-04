// ============================================================
//  [ORCHESTRATOR] 수정 불필요. 5개 stage를 순서대로 조립하는 이음새.
//  각 stage의 실제 알고리즘은 src/modules/tv/ 의 해당 .cpp 에서 채웁니다.
// ============================================================
#include "modules/torque_vectoring.h"
#include "modules/tv/reference.h"
#include "modules/tv/yaw_control.h"
#include "modules/tv/load.h"
#include "modules/tv/traction.h"
#include "modules/tv/allocation.h"
#include <cmath>

TVOutput tv_compute(const TVInput &in, TVYawState &s) {
    // 1) 조향 의도 → 목표 yaw rate
    float desired_yaw = tv_reference_compute(in.steering_angle, in.vehicle_speed, TV_PARAMS);
    // 2) yaw 오차 → 요 모멘트 Mz
    float mz = 0.0f;
    if (std::isfinite(in.vehicle_speed) &&
        std::fabs(in.vehicle_speed) >= TV_PARAMS.tv_min_speed_mps) {
        mz = tv_yaw_compute(desired_yaw, in.yaw_rate, in.dt, TV_PARAMS, s);
    } else {
        // Explicit low-speed disable: reference=0 alone would still let the
        // feedback controller react to measured yaw.
        s = TVYawState{};
    }
    // 3) 가속도 → 바퀴별 수직하중 Fz
    WheelLoads fz = tv_load_compute(in.ax, in.ay, TV_PARAMS);
    // 4) Fz + 마찰원 → 모터별 최대 상전류
    MaxTorque lim = tv_traction_compute(fz, in.ay, TV_PARAMS);
    // 5) 총전류 + Mz, 상한 제약 → 좌/우 상전류 명령
    TVAllocOutput a = tv_alloc_compute(in.total_torque, mz, lim, TV_PARAMS);

    return TVOutput{
        a.torque_L, a.torque_R,
        desired_yaw, mz, fz.fz_L, fz.fz_R, lim.max_L, lim.max_R,
    };
}
