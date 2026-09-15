#include "modules/tv/gate.h"
#include "modules/torque_vectoring.h"
#include <cmath>

TVGate tv_gate_evaluate(const TVInput &in, const TVParams &p) {
    TVGate g{};
    g.driver_switch_on = in.tv_enable_requested;
    // 게인이 전부 0이면 Mz는 어차피 0이지만, 판정을 명시해야 계기판이 "튜닝 전"과
    // "스위치 OFF"를 구분해 표시할 수 있다.
    g.gains_enabled = std::fabs(p.kp) > 1.0e-6f ||
                      std::fabs(p.ki) > 1.0e-6f ||
                      std::fabs(p.kd) > 1.0e-6f;
    g.imu_valid   = in.imu_valid;
    g.speed_valid = in.vehicle_speed_valid;
    // 부호는 주행 방향이고 TV 임계는 크기로 판단한다. NaN은 두 비교 모두 false라
    // 자동으로 차단된다.
    g.speed_above_min = std::isfinite(in.vehicle_speed) &&
                        std::fabs(in.vehicle_speed) >= p.tv_min_speed_mps;

    g.active = g.driver_switch_on && g.gains_enabled && g.imu_valid &&
               g.speed_valid && g.speed_above_min;
    return g;
}
