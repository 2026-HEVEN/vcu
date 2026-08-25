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
    const bool gains_enabled = std::fabs(TV_PARAMS.kp) > 1.0e-6f ||
                               std::fabs(TV_PARAMS.ki) > 1.0e-6f ||
                               std::fabs(TV_PARAMS.kd) > 1.0e-6f;
    const bool speed_enabled = std::isfinite(in.vehicle_speed) &&
        std::fabs(in.vehicle_speed) >= TV_PARAMS.tv_min_speed_mps;
    const bool control_enabled = gains_enabled && speed_enabled && in.tv_enable_requested;

    float mz = 0.0f;
    if (control_enabled) {
        mz = tv_yaw_compute(desired_yaw, in.yaw_rate, in.dt, TV_PARAMS, s);
    } else {
        // Strict 50:50 OFF whenever any gate fails: kp=ki=kd=0 (gains),
        // low/invalid speed, or the dash TC/TV switch is off
        // (in.tv_enable_requested, wired from state.tv_enable_requested,
        // decoded from Cluster's CAN_ID_CLUSTER_CMD). Always clear history
        // at any of these boundaries.
        s = TVYawState{};
    }
    // 3) 가속도 → 바퀴별 수직하중 Fz
    WheelLoads fz = tv_load_compute(in.ax, in.ay, TV_PARAMS);
    // 4) Fz + 마찰원 → 모터별 최대 상전류
    MaxTorque lim = tv_traction_compute(fz, in.ay, TV_PARAMS);
    // 5) 총전류 + Mz, 상한 제약 → 좌/우 상전류 명령
    // Strict OFF must be behaviorally identical to the pre-TV 50:50 split.
    // In particular, Stage 4 may still calculate diagnostic limits but must
    // not silently reduce longitudinal demand while TV is disabled.
    TVAllocOutput a{};
    if (control_enabled) {
        a = tv_alloc_compute(in.total_torque, mz, lim, TV_PARAMS);
    } else {
        const float safe_total = std::isfinite(in.total_torque)
            ? in.total_torque : 0.0f;
        const float half = 0.5f * safe_total;
        a = {Amp(half), Amp(half)};
    }

    return TVOutput{
        a.torque_L, a.torque_R,
        desired_yaw, mz, fz.fz_L, fz.fz_R, lim.max_L, lim.max_R,
    };
}
