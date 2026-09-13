#include "modules/drive_supervisor.h"
#include <cmath>
#include "state.h" // state에 접근하여 em_hv_voltage_v, em_hv_current_a 등 활용

namespace {
float clamp01(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

float positive(float value) { return value > 0.0f ? value : 0.0f; }

void scale_drive(float &value, float scale, bool propulsion_requested) {
    if (propulsion_requested) value *= scale;
    else if (value > 0.0f) value *= scale;
}

void reset_rise_limit(DriveSupervisorState &state) {
    state.previous_left_a = 0.0f;
    state.previous_right_a = 0.0f;
}

float limit_rising_magnitude(float target, float previous, float max_step,
                             bool &limited) {
    if (target == 0.0f) return 0.0f;
    const float target_magnitude = std::fabs(target);
    const bool same_direction = target * previous > 0.0f;
    const float previous_magnitude = same_direction
        ? std::fabs(previous) : 0.0f;

    if (target_magnitude <= previous_magnitude) return target;

    const float next_magnitude = std::fmin(
        target_magnitude, previous_magnitude + positive(max_step));
    if (next_magnitude < target_magnitude) limited = true;
    return std::copysign(next_magnitude, target);
}
}

DriveSupervisorOutput drive_supervisor_compute(
    const DriveSupervisorInput &in, const DriveSupervisorParams &params,
    DriveSupervisorState &state_sup) {
    DriveSupervisorOutput out;
    out.left_a = in.requested_left_a;
    out.right_a = in.requested_right_a;

    // 1. 컨트롤러 피드백 끊김 또는 Fault 발생 시 즉시 차단
    if (!in.controller_feedback_fresh || in.controller_fault) {
        out.left_a = 0.0f;
        out.right_a = 0.0f;
        reset_rise_limit(state_sup);
        out.controller_blocked = true;
        return out;
    }

    // 2. [온도 제한 및 불확실한 Paddock 센서 컷오프 로직 전면 삭제됨]
    // 대회 규정 10kW 제한 내에서 탈 일 없으므로, 불필요한 디레이팅을 걷어내어 응답성을 확보합니다.

    // 3. 가속 응답 조정을 위한 Slew Rate Limiter (전류 상승 램프)
    float slew_scale = 1.0f;
    if (in.propulsion_requested) {
        if (params.drive_current_rise_time_s > 0.0f) {
            const float desired_peak = std::fmax(std::fabs(out.left_a),
                                                 std::fabs(out.right_a));
            const float rise_rate_a_per_s =
                positive(params.drive_current_max_per_motor_a) /
                params.drive_current_rise_time_s;
            const float max_step = rise_rate_a_per_s * positive(in.control_dt_s);
            bool slew_limited = false;
            out.left_a = limit_rising_magnitude(
                out.left_a, state_sup.previous_left_a, max_step, slew_limited);
            out.right_a = limit_rising_magnitude(
                out.right_a, state_sup.previous_right_a, max_step, slew_limited);
            if (slew_limited) {
                const float limited_peak = std::fmax(std::fabs(out.left_a),
                                                     std::fabs(out.right_a));
                if (desired_peak > 0.0f)
                    slew_scale = limited_peak / desired_peak;
                out.drive_slew_limited = true;
            }
        }
    }

    // 4. [핵심] 자체 제작 에너지미터 실측 기반 10kW 전력 제한 (Hard-Cut)
    // 기존의 부정확한 모터 RPM/Kt 예측 전력 공식을 버리고, 
    // 에너지미터가 보내준 실제 고전압 팩 전압과 전류를 곱한 실측 전력을 사용합니다.
    
    // state 구조체에서 직접 에너지미터 실측값 가져오기
    out.measured_bus_power_w = std::fabs(state.em_hv_voltage_v * state.em_hv_current_a);
    
    float effective_power_limit_w = params.power_soft_limit_w; // 보통 10000W (10kW) 설정

    float power_scale = 1.0f;
    if (effective_power_limit_w > 0.0f &&
        out.measured_bus_power_w > effective_power_limit_w) {
        // 실측 전력이 10kW를 초과하는 순간 비율대로 전류를 강제 하향 조정
        power_scale = clamp01(effective_power_limit_w / out.measured_bus_power_w);
        scale_drive(out.left_a, power_scale, in.propulsion_requested);
        scale_drive(out.right_a, power_scale, in.propulsion_requested);
        out.power_limited = true;
    }

    // 5. 이전 명령 상태 저장 (출렁임 및 오실레이션 방지)
    if (in.propulsion_requested) {
        state_sup.previous_left_a = out.left_a;
        state_sup.previous_right_a = out.right_a;
    } else {
        reset_rise_limit(state_sup);
    }

    // 최종 적용된 스케일 계산 반환
    out.applied_scale = power_scale * slew_scale;
    return out;
}
