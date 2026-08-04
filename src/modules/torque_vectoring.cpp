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

TVOutput tv_compute(const TVInput &in, TVYawState &s) {
    // 1) 조향 의도 → 목표 yaw rate
    float desired_yaw = tv_reference_compute(in.steering_angle, in.vehicle_speed, TV_PARAMS);
    // 2) yaw 오차 → 요 모멘트 Mz.  저속(§4)에선 TV 전체 차단: Mz=0 + 상태 리셋 → 좌우 대칭.
    float mz;
    if (in.vehicle_speed >= TV_PARAMS.tv_min_speed_mps) {
        mz = tv_yaw_compute(desired_yaw, in.yaw_rate, in.dt, TV_PARAMS, s);
    } else {
        mz = 0.0f;
        s.integral = 0.0f;              // 재활성 시 와인드업 방지
        s.prev_measured = in.yaw_rate;  // 재활성 시 derivative kick 방지
    }
    // 3) 가속도 단위 변환(g→m/s², §2.2) 후 바퀴별 수직하중 Fz
    float ax = in.ax * TV_PARAMS.accel_to_mps2;
    float ay = in.ay * TV_PARAMS.accel_to_mps2;
    WheelLoads fz = tv_load_compute(ax, ay, TV_PARAMS);
    // 4) Fz + 마찰원 → 바퀴별 최대 종토크
    MaxTorque lim = tv_traction_compute(fz, ay, TV_PARAMS);
    // 5) 총전류 + Mz, 상한 제약 → 좌/우 명령 (allocation이 Mz[N·m]→ΔI[A] 환산)
    TVAllocOutput a = tv_alloc_compute(in.total_torque, mz, lim, TV_PARAMS);

    return TVOutput{
        a.torque_L, a.torque_R,
        desired_yaw, mz, fz.fz_L, fz.fz_R, lim.max_L, lim.max_R,
    };
}
