// [FILL-IN] 4륜 휠속도 → 차속 추정
#include "modules/vehicle_speed.h"

// ── 왜 전륜 평균인가 ──────────────────────────────────────────────
//   · 후륜은 구동륜이라 토크를 걸면 슬립해서 실제보다 빠르게 읽힌다 → 차속 추정에 부적합.
//   · ABS/TCS는 "비구동륜 중 빠른 쪽(max)"을 쓰지만 그건 슬립 검출용 보수적 기준속도다.
//     TV의 reference stage는 바이시클 모델에 넣을 CG 속도가 필요하다.
//     선회 중 좌우 전륜은 yaw_rate × track/2 만큼 차이 나므로, max를 쓰면 항상 바깥
//     바퀴를 잡아 차속을 계통적으로 과대평가한다 → 목표 yaw rate가 부풀고 제어기가
//     존재하지 않는 오차를 쫓는다. **평균**을 쓰면 이 성분이 1차로 상쇄된다.
//   · 한쪽만 살아있을 때만 yaw_rate로 명시적 보정을 한다.
//
// ── 샘플 간 급변 검사는 두지 않는다 ─────────────────────────────────
//   예전에는 직전 추정치 대비 tick당 변화량(15 m/s² × 10 ms)으로 바퀴를 기각했다.
//   24 PPR을 10 ms마다 읽으면 필터를 거쳐도 양자화 흔들림이 그 한도를 넘어서,
//   09-27 주행에서 네 바퀴가 멀쩡한데도 주행 프레임의 약 9~26%가 invalid가 됐다.
//   같은 로그에서 바퀴 무효 판정은 0회였다. 바퀴 신뢰성은 드라이버 판정
//   (wheel_valid)만 본다. 후륜 폴백도 슬립이 섞여 쓸 곳이 없어 뺐다.

namespace {
constexpr float PI_F       = 3.14159265f;
constexpr float DEG_TO_RAD = 0.01745329f;

float rpm_to_mps(float rpm, const VehicleSpeedCalib &c) {
    return rpm * (2.0f * PI_F * c.tire_radius_m) / 60.0f;
}
}  // namespace

VehicleSpeedOutput vehicle_speed_compute(const VehicleSpeedInput &in,
                                         const VehicleSpeedCalib &c,
                                         VehicleSpeedState &s) {
    const bool fl_ok = in.wheel_valid[WHEEL_FL];
    const bool fr_ok = in.wheel_valid[WHEEL_FR];
    const float v_fl = rpm_to_mps((float)in.wheel_rpm[WHEEL_FL], c);
    const float v_fr = rpm_to_mps((float)in.wheel_rpm[WHEEL_FR], c);

    // 선회 성분: 좌회전(+yaw)이면 좌측 전륜이 안쪽이라 느리다.
    //   v_FL = v_cg − r·track/2 ,  v_FR = v_cg + r·track/2
    const float yaw_term = (in.yaw_rate * DEG_TO_RAD) * c.track_m * 0.5f;

    VehicleSpeedOutput out{ s.speed_mps, false };

    if (fl_ok && fr_ok) {
        // 정상: 좌우 평균 (yaw 성분이 서로 상쇄된다)
        out.speed_mps = (v_fl + v_fr) * 0.5f;
        out.valid = true;
    } else if (fl_ok) {
        out.speed_mps = v_fl + yaw_term;         // 한쪽만 살아있음 → yaw 보정한 단일값
        out.valid = true;
    } else if (fr_ok) {
        out.speed_mps = v_fr - yaw_term;
        out.valid = true;
    }
    // 둘 다 무효: 직전 값을 유지하고 valid=false (표시에만 쓰고 TV는 쓰지 않는다)

    if (!(out.speed_mps >= 0.0f)) out.speed_mps = 0.0f;   // 음수·NaN 차단. 휠속 센서는 방향을 모른다

    s.speed_mps = out.speed_mps;
    return out;
}
