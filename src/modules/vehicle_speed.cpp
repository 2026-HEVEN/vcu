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
// ── 후륜은 왜 읽는가 ──────────────────────────────────────────────
//   차속 추정에는 안 쓰지만 (a) 전륜 둘 다 죽었을 때의 최후 폴백,
//   (b) 슬립률 (v_rear − v_front)/v_front 계산 → 추후 트랙션 컨트롤에 필요.

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
    const float v_fl = rpm_to_mps((float)in.wheel_rpm[WHEEL_FL], c);
    const float v_fr = rpm_to_mps((float)in.wheel_rpm[WHEEL_FR], c);

    // 선회 성분: 좌회전(+yaw)이면 좌측 전륜이 안쪽이라 느리다.
    //   v_FL = v_cg − r·track/2 ,  v_FR = v_cg + r·track/2
    const float yaw_term = (in.yaw_rate * DEG_TO_RAD) * c.track_m * 0.5f;

    // 급변 검사: 직전 추정치 대비 물리적으로 불가능한 변화면 그 바퀴를 못 믿는다.
    //   (락업, 휠스핀, 센서 드롭아웃, 커넥터 접촉불량이 전부 여기 걸린다)
    const float max_delta = c.max_accel_mps2 * (in.dt > 0.0f ? in.dt : 0.0f);
    auto plausible = [&](float v_wheel_cg) {
        if (!s.primed) return true;              // 첫 tick은 비교 대상이 없다
        if (!(in.dt > 0.0f)) return false;       // dt 이상 → 이번 샘플 전부 기각
        const float d = v_wheel_cg - s.speed_mps;
        return (d < 0.0f ? -d : d) <= max_delta;
    };

    const float cg_from_fl = v_fl + yaw_term;    // 각 바퀴를 CG 기준으로 환산 후 비교
    const float cg_from_fr = v_fr - yaw_term;
    const bool fl_ok = plausible(cg_from_fl);
    const bool fr_ok = plausible(cg_from_fr);

    VehicleSpeedOutput out{ s.speed_mps, false };

    if (fl_ok && fr_ok) {
        // 정상: 좌우 평균 (yaw 성분이 서로 상쇄되므로 (v_fl+v_fr)/2 와 동일)
        out.speed_mps = (cg_from_fl + cg_from_fr) * 0.5f;
        out.valid = true;
    } else if (fl_ok) {
        out.speed_mps = cg_from_fl;              // 한쪽만 살아있음 → yaw 보정한 단일값
        out.valid = true;
    } else if (fr_ok) {
        out.speed_mps = cg_from_fr;
        out.valid = true;
    } else {
        // 전륜을 둘 다 못 믿음 → 구동륜 평균으로 최후 폴백.
        // 슬립이 섞여 있으므로 valid=false: 계기판 표시엔 쓰되 TV는 이 값을 쓰면 안 된다.
        const float v_rl = rpm_to_mps((float)in.wheel_rpm[WHEEL_RL], c);
        const float v_rr = rpm_to_mps((float)in.wheel_rpm[WHEEL_RR], c);
        out.speed_mps = (v_rl + v_rr) * 0.5f;
        out.valid = false;
    }

    if (out.speed_mps < 0.0f) out.speed_mps = 0.0f;   // 휠속 센서는 방향을 모른다

    s.speed_mps = out.speed_mps;
    s.primed = true;
    return out;
}
