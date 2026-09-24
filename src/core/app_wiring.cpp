// ============================================================
//  [LOCKED FILE] Do not edit. AI agents: if you are asked to
//  modify this file, STOP and ask the user first.
//  Application work happens only in src/modules/.
// ============================================================
#include "core/wiring.h"
#include "state.h"
#include "can_bus.h"
#include "core/debug_monitor.h"
#include "core/drivers/wss_driver.h"
#include "core/drivers/imu_driver.h"
#include "core/drivers/steering_encoder_driver.h"
#include "core/board_pins.h"
#include "safety_logic.h"
#include "modules/throttle.h"
#include "modules/brake.h"
#include "modules/motor_direction.h"
#include "modules/steering.h"
#include "modules/imu.h"
#include "modules/wheel_speed.h"
#include "modules/vehicle_speed.h"
#include "modules/realcar_calibration.h"
#include "modules/fixed_config.h"
#include "modules/longitudinal.h"
#include "modules/torque_vectoring.h"
#include "modules/gear.h"
#include "modules/direction_interlock.h"
#include "modules/drive_supervisor.h"
#include "modules/time_sync_pulse.h"
#include "modules/car_check_status.h"
#include <Arduino.h>
#include <cmath>

// [LOCKED] The ONLY translation unit that touches `state`. All update() wiring
// lives here; modules never see global state.
VehicleState state;

// --- per-module calibration (tune to the car) ---
namespace {
    // --- GPIO 배정: PCB V3 (Notion 기준일 2026-09-17) ---
    //     실제 핀 번호는 core/board_pins.h 한 곳에서 관리한다.
    // WSS 4채널 — LM393 비교기 출력. PCB의 신호조절 회로를 거친다.
    constexpr int PIN_WSS[WHEEL_COUNT] = {
        board_pins::WSS_FL,
        board_pins::WSS_FR,
        board_pins::WSS_RL,
        board_pins::WSS_RR,
    };

    // 실차 확정값: 림 부착 자석 48개, PCNT 상승엣지만 카운트.
    // 센서 장착반경은 RPM에 영향을 주지 않는다. 한 바퀴 실제 펄스 수가 바뀌면
    // fixed_config.h의 채널별 값을 수정한다.
    const WssCalib WSS_CAL[WHEEL_COUNT] = {
        { fixed_config::vehicle::WSS_PULSES_PER_WHEEL_REV_FL,
          realcar_cal::provisional::WSS_FILTER_TIME_CONSTANT_S },
        { fixed_config::vehicle::WSS_PULSES_PER_WHEEL_REV_FR,
          realcar_cal::provisional::WSS_FILTER_TIME_CONSTANT_S },
        { fixed_config::vehicle::WSS_PULSES_PER_WHEEL_REV_RL,
          realcar_cal::provisional::WSS_FILTER_TIME_CONSTANT_S },
        { fixed_config::vehicle::WSS_PULSES_PER_WHEEL_REV_RR,
          realcar_cal::provisional::WSS_FILTER_TIME_CONSTANT_S },
    };
    WheelSpeedFilterState wheel_speed_filter_state[WHEEL_COUNT]{};
    // Independent display filter: rejecting corrupt telemetry must not change control.
    WheelSpeedFilterState wheel_telemetry_filter[WHEEL_COUNT]{};
    const VehicleSpeedCalib VSPEED_CAL{};   // 값은 realcar_calibration.h에서 관리
    VehicleSpeedState vspeed_state{};
    const SteerCalib STEER_CAL {
        static_cast<uint16_t>(realcar_cal::provisional::STEERING_CENTER_COUNTS),
        realcar_cal::provisional::STEERING_COUNTS_PER_UNIT,
        realcar_cal::provisional::STEERING_INVERT,
    };
    TVYawState       tv_yaw_state{};       // yaw 제어기 이력 (전역상태 아님, 여기서만 보유)
    DriveMode        drive_mode = DriveMode::Normal;
    constexpr float  TV_DT_S = fixed_config::vehicle::CONTROL_PERIOD_S;
    constexpr float  WHEEL_SPEED_DT_S = fixed_config::vehicle::CONTROL_PERIOD_S;
    const GearCalib GEAR_CAL {
        (uint16_t)realcar_cal::bringup::GEAR_REVERSE_ADC,
        (uint16_t)realcar_cal::bringup::GEAR_DRIVE_ADC,
    };
    GearFilterState gear_filter_state{};
    DirectionInterlockState direction_interlock_state{};
    DriveSupervisorState drive_supervisor_state{};
    const DriveSupervisorParams DRIVE_SUPERVISOR_PARAMS {
        realcar_cal::bringup::DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A,
        realcar_cal::bringup::DRIVE_CURRENT_RISE_TIME_S,
        realcar_cal::bringup::ENABLE_DRIVE_POWER_LIMIT
            ? realcar_cal::bringup::DRIVE_POWER_SOFT_LIMIT_W : 0.0f,
        realcar_cal::bringup::DRIVETRAIN_EFFICIENCY,
        fixed_config::vehicle::MOTOR_KT_NM_PER_A,
        realcar_cal::bringup::PADDOCK_CURRENT_ZERO_SPEED_PER_MOTOR_A,
        realcar_cal::bringup::PADDOCK_CURRENT_HIGH_SPEED_PER_MOTOR_A,
        realcar_cal::bringup::PADDOCK_CURRENT_LINEAR_END_SPEED_MPS,
        realcar_cal::bringup::PADDOCK_POWER_SOFT_LIMIT_W,
        realcar_cal::bringup::PADDOCK_CONTROLLER_BUS_CURRENT_LIMIT_A,
        realcar_cal::bringup::PADDOCK_PACK_CURRENT_LIMIT_A,
        fixed_config::runtime::TELEMETRY_TEMPERATURE_VALID_MIN_C,
        realcar_cal::bringup::PADDOCK_REQUIRE_PACK_DATA,
        realcar_cal::bringup::CONTROLLER_DERATE_START_C,
        realcar_cal::bringup::CONTROLLER_CUTOFF_C,
        realcar_cal::bringup::MOTOR_DERATE_START_C,
        realcar_cal::bringup::MOTOR_CUTOFF_C,
    };
    const TimeSyncPulseParams TIME_SYNC_PARAMS {
        fixed_config::bench::ENABLE_TIME_SYNC_PULSE,
        fixed_config::bench::TIME_SYNC_PHASE_CURRENT_PER_MOTOR_A,
        fixed_config::bench::TIME_SYNC_PULSE_ON_S,
        fixed_config::bench::TIME_SYNC_PULSE_OFF_S,
        fixed_config::bench::TIME_SYNC_PULSE_COUNT,
        fixed_config::bench::TIME_SYNC_ARM_TIMEOUT_S,
    };
    TimeSyncPulseState time_sync_state{};
    TimeSyncPulseOutput time_sync_output{};
    uint32_t motor_command_seq = 0U;   // starts at 1; seq 0 = nothing published
}

static void throttle_update() {
    state.throttle_raw_adc = analogRead(board_pins::THROTTLE_ADC);
    if (state.throttle_raw_adc < state.throttle_window_min)
        state.throttle_window_min = (uint16_t)state.throttle_raw_adc;
    state.throttle_signal_valid =
        state.throttle_raw_adc >=
            (int)realcar_cal::bringup::THROTTLE_SIGNAL_VALID_MIN_ADC;
    if (!state.throttle_signal_valid) {
        state.throttle_last_invalid_raw = (uint16_t)state.throttle_raw_adc;
        if (state.throttle_invalid_samples != UINT16_MAX) ++state.throttle_invalid_samples;
    }
    state.throttle_pct = state.throttle_signal_valid
        ? throttle_compute({ state.throttle_raw_adc }) : Percent(0.0f);
}
static void brake_update() {
    // PCB V3 routes the 12 V brake ON/OFF signal through a divider to an ADC1
    // input. Keep the path disabled until the installed high/low raw values are
    // verified; brake_compute converts the 12-bit ADC value to 0..100%.
    const int raw = realcar_cal::bringup::BRAKE_SENSOR_INSTALLED
        ? analogRead(board_pins::BRAKE_ONOFF_ADC)
        : 0;
    BrakeOutput o = brake_compute({ raw });
    state.brake_pct = o.pct; state.brake_active = o.active;
}
static void steering_update() {
    const SteerRaw raw = steering_encoder_driver::read();
    state.steering_angle = steering_compute(raw, STEER_CAL);
    state.steering_telemetry.unit = (float)state.steering_angle;
    state.steering_telemetry.valid = raw.counts <= 16380U &&
        std::isfinite(state.steering_telemetry.unit);
}
static void imu_update() {
    ImuOutput o = imu_compute(imu_driver::read());
    state.imu_valid = !imu_driver::stale();
    state.yaw_rate = state.imu_valid ? o.yaw_rate : 0.0f;
    state.accel_x = state.imu_valid ? o.accel_x : 0.0f;
    state.accel_y = state.imu_valid ? o.accel_y : 0.0f;
    state.imu_telemetry.yaw_dps = o.yaw_rate;
    state.imu_telemetry.ax_g = o.accel_x;
    state.imu_telemetry.ay_g = o.accel_y;
    state.imu_telemetry.yaw_valid = imu_driver::yaw_sample_fresh() && std::isfinite(o.yaw_rate);
    state.imu_telemetry.accel_valid = imu_driver::accel_sample_fresh() &&
        std::isfinite(o.accel_x) && std::isfinite(o.accel_y);
}
static void wheel_speed_update() {
    for (int ch = 0; ch < WHEEL_COUNT; ++ch) {
        const WssReading reading = wss_driver::read(ch);
        const bool valid = car_check_wheel_sample_valid(wss_driver::last_read_ok(ch),
            reading.pulse_delta, reading.dt_ms, WSS_CAL[ch].pulses_per_rev);
        state.wheel_telemetry.valid[ch] = valid;
        if (valid) {
            state.wheel_pulse_total[ch] += reading.pulse_delta;
            state.wheel_speed[ch] = wheel_speed_compute_filtered(
                reading, WSS_CAL[ch], wheel_speed_filter_state[ch]);
            const float rpm = (float)wheel_speed_compute_filtered(
                reading, WSS_CAL[ch], wheel_telemetry_filter[ch]);
            state.wheel_telemetry.kph[ch] = rpm * 6.283185307f *
                realcar_cal::provisional::WHEEL_SPEED_ROLLING_RADIUS_M * 0.06f;
        } else {
            wheel_telemetry_filter[ch] = WheelSpeedFilterState{};
            state.wheel_telemetry.kph[ch] = 0.0f;
        }
    }
}
static void vehicle_speed_update() {
    VehicleSpeedInput in{};
    for (int ch = 0; ch < WHEEL_COUNT; ++ch) in.wheel_rpm[ch] = state.wheel_speed[ch];
    in.yaw_rate = state.yaw_rate;
    in.dt       = WHEEL_SPEED_DT_S;
    VehicleSpeedOutput o = vehicle_speed_compute(in, VSPEED_CAL, vspeed_state);
    state.vehicle_speed_mps   = o.speed_mps;
    state.vehicle_speed_valid = o.valid;
}
static void gear_update_task() {
    // Read the raw ladder on every build so the wiring can be checked even
    // when selector authority is temporarily disabled.
    state.gear_raw_adc = (uint16_t)analogRead(board_pins::GEAR_ADC);
    state.gear_sensed = gear_update(state.gear_raw_adc, GEAR_CAL,
                                    realcar_cal::bringup::GEAR_STABLE_SAMPLES,
                                    gear_filter_state);
    if (!realcar_cal::bringup::GEAR_SELECTOR_INSTALLED) {
        // Legacy fallback for a build without a connected selector.
        state.gear = Gear::Drive;
        return;
    }
    state.gear = state.gear_sensed;
}
static void paddock_update() {
    if (!state.cluster_cmd_alive) {
        // Do not unexpectedly remove an already-active limit when the dash
        // disappears. At boot the default remains inactive.
        return;
    }
    if (!state.paddock_requested) {
        state.paddock_active = false;
        return;
    }
    if (!state.paddock_active &&
        state.vehicle_speed_mps <= realcar_cal::bringup::PADDOCK_ENTRY_SPEED_MAX_MPS &&
        (float)state.throttle_pct <= fixed_config::runtime::THROTTLE_ARM_MAX_PCT) {
        state.paddock_active = true;
    }
}
static void longitudinal_update() {
    static RegenReleaseState release_state{};
    const bool released_for_regen = regen_release_update(
        (float)state.throttle_pct, state.throttle_signal_valid, release_state);
    const bool forward_for_regen = regen_forward_rotation_ok(
        state.controller_fb1_L.motor_speed_rpm,
        state.controller_fb1_R.motor_speed_rpm,
        state.controller_feedback_fresh_L && state.controller_feedback_fresh_R,
        realcar_cal::bringup::REGEN_MIN_FORWARD_RPM);
    state.total_torque = longitudinal_compute({
        (float)state.throttle_pct, state.pack_soc, drive_mode,
        released_for_regen && state.regen_auto_requested &&
            realcar_cal::bringup::REGEN_HARDWARE_VALIDATED &&
            state.gear == Gear::Drive && state.pack_data_valid &&
            forward_for_regen });
    state.longitudinal_regen_demand = state.total_torque < 0.0f;
    const bool throttle_released =
        (float)state.throttle_pct <= fixed_config::runtime::THROTTLE_ARM_MAX_PCT;
    const bool stopped =
        abs(state.controller_fb1_L.motor_speed_rpm) <=
            realcar_cal::bringup::GEAR_DIRECTION_CHANGE_MAX_RPM &&
        abs(state.controller_fb1_R.motor_speed_rpm) <=
            realcar_cal::bringup::GEAR_DIRECTION_CHANGE_MAX_RPM;
    const DirectionInterlockOutput direction = direction_interlock_update(
        state.gear, throttle_released, stopped,
        realcar_cal::bringup::GEAR_DIRECTION_ARM_SAMPLES,
        direction_interlock_state);
    state.propulsion_direction_armed = direction.propulsion_enabled;
    state.total_torque = directional_current(
        state.total_torque, state.gear, direction.propulsion_enabled);
}
static void torque_vectoring_update() {
    // 게이트 조건을 여기서 미리 접지 않는다. 원본 값과 유효성 플래그를 그대로
    // 넘기고, 판정은 tv_gate_evaluate()가 단독으로 한다.
    const TVInput tv_in{
        Ampere{state.total_torque}, DegPerSec{state.yaw_rate},
        state.steering_angle,
        Mps{state.vehicle_speed_mps},
        GForce{state.accel_x}, GForce{state.accel_y}, Seconds{TV_DT_S},
        state.tv_enable_requested, state.vehicle_speed_valid, state.imu_valid
    };
    TVOutput o = tv_compute(tv_in, tv_yaw_state);
    state.tv_gate = o.gate;
    state.tv_pipeline_active = o.gate.active;
    state.requested_torque_L = o.torque_L;
    state.requested_torque_R = o.torque_R;
    // 중간신호 관측용 복사 (debug_monitor / Cluster에서 튜닝에 사용)
    // state는 텔레메트리/CAN 인코딩용이라 생 float로 유지한다. 여기가 파이프라인
    // 밖으로 나가는 경계다.
    state.desired_yaw_rate = (float)o.desired_yaw_rate;
    state.yaw_moment       = (float)o.yaw_moment;
    state.fz_L = (float)o.fz_L; state.fz_R = (float)o.fz_R;
    state.max_torque_L = (float)o.max_torque_L;
    state.max_torque_R = (float)o.max_torque_R;
}
static void time_sync_pulse_update() {
    const bool throttle_released =
        (float)state.throttle_pct <= fixed_config::runtime::THROTTLE_ARM_MAX_PCT;
    const bool mode_requests_off = !state.tv_enable_requested &&
        !state.regen_auto_requested && !state.paddock_active &&
        !state.paddock_requested;
    const bool runtime_ok = can_bus::handshaked() && torque_allowed() &&
        state.throttle_signal_valid && state.controller_feedback_fresh &&
        !state.controller_fault_latched &&
        throttle_released && !state.brake_active &&
        state.gear == Gear::Drive && mode_requests_off &&
        state.vehicle_speed_valid &&
        state.vehicle_speed_mps <=
            fixed_config::bench::TIME_SYNC_START_SPEED_MAX_MPS;
    const bool start_ok = runtime_ok;

    time_sync_output = time_sync_pulse_step({
        debug_consume_time_sync_arm_request(),
        debug_consume_time_sync_run_request(),
        debug_consume_time_sync_cancel_request(),
        start_ok,
        runtime_ok,
        fixed_config::vehicle::CONTROL_PERIOD_S,
    }, TIME_SYNC_PARAMS, time_sync_state);

    state.time_sync_armed = time_sync_output.armed;
    state.time_sync_active = time_sync_output.running;
    state.time_sync_command_a = time_sync_output.left_a;
    if (time_sync_output.completed_event) ++state.time_sync_completed_count;
    if (time_sync_output.aborted_event) ++state.time_sync_aborted_count;
}
static void drive_supervisor_update() {
    const float requested_left_a = time_sync_output.override_active
        ? time_sync_output.left_a : (float)state.requested_torque_L;
    const float requested_right_a = time_sync_output.override_active
        ? time_sync_output.right_a : (float)state.requested_torque_R;
    constexpr float TWO_PI_OVER_60 = 0.104719755f;
    const float motor_speed_mps =
        std::fmax(std::fabs((float)state.controller_fb1_L.motor_speed_rpm),
                  std::fabs((float)state.controller_fb1_R.motor_speed_rpm)) *
        TWO_PI_OVER_60 *
        realcar_cal::provisional::WHEEL_SPEED_ROLLING_RADIUS_M /
        fixed_config::vehicle::GEAR_RATIO;
    state.paddock_speed_mps =
        std::fmax(state.vehicle_speed_mps, motor_speed_mps);
    const bool propulsion_requested =
        !time_sync_output.override_active &&
        state.propulsion_direction_armed && state.throttle_signal_valid &&
        (float)state.throttle_pct > 0.0f;
    const DriveSupervisorInput in {
        requested_left_a, requested_right_a,
        state.controller_feedback_fresh,
        state.controller_fault_latched ||
            state.controller_fb2_L.speed_mode || state.controller_fb2_R.speed_mode ||
            !torque_allowed(),
        state.controller_fb1_L.bus_voltage_v, state.controller_fb1_R.bus_voltage_v,
        state.controller_fb1_L.bus_current_a, state.controller_fb1_R.bus_current_a,
        state.controller_fb1_L.phase_current_a, state.controller_fb1_R.phase_current_a,
        state.controller_fb1_L.motor_speed_rpm, state.controller_fb1_R.motor_speed_rpm,
        (float)state.controller_fb2_L.controller_temp_c,
        (float)state.controller_fb2_R.controller_temp_c,
        (float)state.controller_fb2_L.motor_temp_c,
        (float)state.controller_fb2_R.motor_temp_c,
        state.paddock_active,
        propulsion_requested, fixed_config::vehicle::CONTROL_PERIOD_S,
        state.vehicle_speed_mps, state.paddock_speed_mps,
        state.pack_data_valid, state.pack_current_a,
    };
    const DriveSupervisorOutput out =
        drive_supervisor_compute(in, DRIVE_SUPERVISOR_PARAMS,
                                 drive_supervisor_state);
    // drive_supervisor는 생 float[A]로 계산한다. 여기가 그 값이 모터 명령
    // 차원으로 확정되는 경계다 -- Amp(...)로 의도를 명시한다.
    state.torque_L = Amp{out.left_a};
    state.torque_R = Amp{out.right_a};
    state.measured_bus_power_w = out.measured_bus_power_w;
    state.estimated_input_power_w = out.estimated_input_power_w;
    state.predicted_command_power_w = out.predicted_command_power_w;
    state.paddock_current_limit_a = out.paddock_current_limit_a;
    state.drive_limit_scale = out.applied_scale;
    state.power_limited = out.power_limited;
    state.thermal_limited = out.thermal_limited;
    state.paddock_sensor_blocked = out.paddock_sensor_blocked;
    state.paddock_current_limited = out.paddock_current_limited;
    state.drive_slew_limited = out.drive_slew_limited;

    // Publish this tick's decision as one unit. safety_task runs BEFORE this
    // task, so torque_allowed() here is this tick's verdict, not the previous.
    MotorCommandSnapshot command_snapshot;
    if (++motor_command_seq == 0U) ++motor_command_seq; // zero is unpublished
    command_snapshot.seq = motor_command_seq;
    command_snapshot.published_ms = millis();
    command_snapshot.left_a = out.left_a;
    command_snapshot.right_a = out.right_a;
    command_snapshot.gear = state.gear;
    command_snapshot.safety_allow = torque_allowed();
    if (!state.controller_feedback_fresh) command_snapshot.block_reasons |= BLOCK_FEEDBACK;
    if (state.controller_fault_latched) command_snapshot.block_reasons |= BLOCK_FAULT;
    if (state.controller_fb2_L.speed_mode || state.controller_fb2_R.speed_mode)
        command_snapshot.block_reasons |= BLOCK_SPEED_MODE;
    if (out.paddock_sensor_blocked) command_snapshot.block_reasons |= BLOCK_PADDOCK_SENSOR;
    if (out.thermal_limited && out.left_a == 0.0f && out.right_a == 0.0f &&
        (requested_left_a != 0.0f || requested_right_a != 0.0f))
        command_snapshot.block_reasons |= BLOCK_THERMAL;
    command_snapshot.throttle_signal_valid = state.throttle_signal_valid;
    command_snapshot.propulsion_direction_armed =
        state.propulsion_direction_armed;
    command_snapshot.regen_allowed = realcar_cal::bringup::REGEN_HARDWARE_VALIDATED &&
        state.regen_auto_requested && state.pack_data_valid &&
        state.throttle_signal_valid && (float)state.throttle_pct == 0.0f;
    // Installed motor polarity confirmed from the 2026-09-05 and 2026-09-21
    // vehicle logs: forward rotation is left +RPM and right -RPM.  Reuse the
    // dedicated regen threshold so zero-speed noise cannot enable regen.
    command_snapshot.forward_rotation = regen_forward_rotation_ok(
        state.controller_fb1_L.motor_speed_rpm,
        state.controller_fb1_R.motor_speed_rpm,
        state.controller_feedback_fresh_L && state.controller_feedback_fresh_R,
        realcar_cal::bringup::REGEN_MIN_FORWARD_RPM);
    can_bus::publish_motor_command(command_snapshot);

    // Record every 10ms control verdict as well as the latest 50ms TX verdict.
    // Thus one short invalid ADC sample is not lost by the slow CAN logger.
    uint16_t observed = command_snapshot.block_reasons;
    if (!command_snapshot.safety_allow) observed |= BLOCK_SAFETY;
    if (!command_snapshot.throttle_signal_valid) observed |= BLOCK_THROTTLE;
    if (!command_snapshot.propulsion_direction_armed) observed |= BLOCK_DIRECTION;
    observed |= can_bus::motor_tx_diagnostics().requested.block_reasons;
    if (observed && !state.diagnostic_block_reasons) {
        state.first_block_reasons = observed;
        state.first_block_ms = millis();
        if (state.block_event_count != UINT16_MAX) ++state.block_event_count;
    }
    state.diagnostic_block_reasons = observed;

    can_bus::note_command();
}
static void can_rx_update()  { can_bus::poll_rx(); }
static void vehicle_speed_can_tx_update() { can_bus::send_vehicle_speed(); }
static void log_can_tx_update() { can_bus::send_log_frames(); }
static void clamp_stats_can_tx_update() { can_bus::send_clamp_stats(); }
static void cluster_status_can_tx_update() { can_bus::send_cluster_status(); }
static void sensor_telemetry_can_tx_update() { can_bus::send_sensor_telemetry(); }
static void safety_task()    { safety_update(); }
static void drive_diagnostics_update() { can_bus::send_drive_diagnostics(); }

// --- task table: add a new module here (one line) ---
Task g_tasks[] = {
    { can_rx_update,            5, 0 },   // 200 Hz drain; feedback precedes control
    { throttle_update,         10, 0 },   // 100 Hz
    { brake_update,            10, 0 },
    { steering_update,         10, 0 },
    { imu_update,              10, 0 },
    { wheel_speed_update,      10, 0 },
    { vehicle_speed_update,    10, 0 },   // 반드시 wheel_speed 다음
    { gear_update_task,        10, 0 },
    { paddock_update,          10, 0 },
    { longitudinal_update,     10, 0 },
    { torque_vectoring_update, 10, 0 },
    { time_sync_pulse_update,  10, 0 },
    // safety BEFORE drive_supervisor: the command snapshot is published at the
    // end of drive_supervisor_update and must carry this tick's safety verdict,
    // not the previous tick's. Do not reorder these two.
    { safety_task,             10, 0 },
    { drive_supervisor_update, 10, 0 },
    { vehicle_speed_can_tx_update, 50, 0 }, // 20 Hz VCU -> Cluster/TMA-1 single speed telemetry
    { log_can_tx_update,       10, 0 },   // 100 Hz VCU -> Monolith 고속 로깅 (Prio 7)
    { clamp_stats_can_tx_update, 1000, 0 }, // 1 Hz Amp 포화 통계 (Prio 7)
    { drive_diagnostics_update, 100, 0 }, // 3 nonblocking frames, 30 frames/s
    { cluster_status_can_tx_update, 50, 0 }, // 20 Hz gear/brake/HV display status
    { sensor_telemetry_can_tx_update, car_check::PERIOD_MS, 0 }, // steering/IMU/WSS/control diagnostics
    { debug_update,            50, 0 },   // 20 Hz compact test log; 1 Hz idle summary
};
const int G_TASK_COUNT = sizeof(g_tasks) / sizeof(g_tasks[0]);

void modules_init() {
    analogReadResolution(12);
    pinMode(board_pins::THROTTLE_ADC, INPUT);
    if (realcar_cal::bringup::BRAKE_SENSOR_INSTALLED) {
        pinMode(board_pins::BRAKE_ONOFF_ADC, INPUT);
    }
    pinMode(board_pins::GEAR_ADC, INPUT);  // gear-ladder 모듈용 예약 입력
    for (int ch = 0; ch < WHEEL_COUNT; ++ch) wss_driver::begin(ch, PIN_WSS[ch]);
    imu_driver::begin();
    steering_encoder_driver::begin();
    can_bus::begin();
}
