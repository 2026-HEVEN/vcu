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
#include "modules/throttle.h"
#include "modules/brake.h"
#include "modules/steering.h"
#include "modules/imu.h"
#include "modules/wheel_speed.h"
#include "modules/vehicle_speed.h"
#include "modules/realcar_calibration.h"
#include "modules/longitudinal.h"
#include "modules/torque_vectoring.h"
#include <Arduino.h>

// [LOCKED] The ONLY translation unit that touches `state`. All update() wiring
// lives here; modules never see global state.
VehicleState state;

// --- per-module calibration (tune to the car) ---
namespace {
    // --- GPIO 배정: 제작 완료 PCB / 하네스 v5 / origin/GPIO-fixed 기준 ---
    //     실제 핀 번호는 core/board_pins.h 한 곳에서 관리한다.
    // WSS 4채널 — 전부 input-only 핀. LM393 오픈컬렉터라 외부 3.3V 풀업 필수.
    constexpr int PIN_WSS[WHEEL_COUNT] = {
        board_pins::WSS_FL,
        board_pins::WSS_FR,
        board_pins::WSS_RL,
        board_pins::WSS_RR,
    };

    // 실차 확정값: 림 부착 자석 48개, PCNT 상승엣지만 카운트.
    // 센서 장착반경은 RPM에 영향을 주지 않는다. 한 바퀴 실제 펄스 수가 바뀌면
    // realcar_calibration.h의 채널별 값을 수정한다.
    const WssCalib WSS_CAL[WHEEL_COUNT] = {
        { realcar_cal::confirmed::WSS_PULSES_PER_WHEEL_REV_FL,
          realcar_cal::provisional::WSS_FILTER_TIME_CONSTANT_S },
        { realcar_cal::confirmed::WSS_PULSES_PER_WHEEL_REV_FR,
          realcar_cal::provisional::WSS_FILTER_TIME_CONSTANT_S },
        { realcar_cal::confirmed::WSS_PULSES_PER_WHEEL_REV_RL,
          realcar_cal::provisional::WSS_FILTER_TIME_CONSTANT_S },
        { realcar_cal::confirmed::WSS_PULSES_PER_WHEEL_REV_RR,
          realcar_cal::provisional::WSS_FILTER_TIME_CONSTANT_S },
    };
    WheelSpeedFilterState wheel_speed_filter_state[WHEEL_COUNT]{};
    const VehicleSpeedCalib VSPEED_CAL{};   // 값은 realcar_calibration.h에서 관리
    VehicleSpeedState vspeed_state{};
    const SteerCalib STEER_CAL {
        static_cast<uint16_t>(realcar_cal::provisional::STEERING_CENTER_COUNTS),
        realcar_cal::provisional::STEERING_COUNTS_PER_UNIT,
        realcar_cal::provisional::STEERING_INVERT,
    };
    TVYawState       tv_yaw_state{};       // yaw 제어기 이력 (전역상태 아님, 여기서만 보유)
    DriveMode        drive_mode = DriveMode::Normal;
    constexpr float  TV_DT_S = realcar_cal::confirmed::CONTROL_PERIOD_S;
    constexpr float  WHEEL_SPEED_DT_S = realcar_cal::confirmed::CONTROL_PERIOD_S;
}

static void throttle_update() {
    state.throttle_pct = throttle_compute({ analogRead(board_pins::THROTTLE_ADC) });
}
static void brake_update() {
    // PCB가 12 V 브레이크 신호를 3.3 V 디지털 입력으로 변환한다.
    const int raw = digitalRead(board_pins::BRAKE_DIGITAL) == HIGH ? 4095 : 0;
    BrakeOutput o = brake_compute({ raw });
    state.brake_pct = o.pct; state.brake_active = o.active;
}
static void steering_update() {
    state.steering_angle = steering_compute(steering_encoder_driver::read(), STEER_CAL);
}
static void imu_update() {
    ImuOutput o = imu_compute(imu_driver::read());
    state.yaw_rate = o.yaw_rate; state.accel_x = o.accel_x; state.accel_y = o.accel_y;
}
static void wheel_speed_update() {
    for (int ch = 0; ch < WHEEL_COUNT; ++ch) {
        state.wheel_speed[ch] = wheel_speed_compute_filtered(
            wss_driver::read(ch), WSS_CAL[ch], wheel_speed_filter_state[ch]);
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
static void longitudinal_update() {
    state.total_torque = longitudinal_compute({
        state.throttle_pct, state.brake_pct, state.pack_soc, drive_mode });
}
static void torque_vectoring_update() {
    const TVInput tv_in{
        state.total_torque, state.yaw_rate, state.steering_angle,
        // 전륜 신호를 못 믿으면 차속 0 → reference stage의 저속 컷오프에 걸려 TV가 꺼진다.
        state.vehicle_speed_valid ? state.vehicle_speed_mps : 0.0f,
        state.accel_x, state.accel_y, TV_DT_S,
        state.tv_enable_requested
    };
    TVOutput o = tv_compute(tv_in, tv_yaw_state);
    state.torque_L = o.torque_L; state.torque_R = o.torque_R;
    // 중간신호 관측용 복사 (debug_monitor / Cluster에서 튜닝에 사용)
    state.desired_yaw_rate = o.desired_yaw_rate;
    state.yaw_moment       = o.yaw_moment;
    state.fz_L = o.fz_L; state.fz_R = o.fz_R;
    state.max_torque_L = o.max_torque_L; state.max_torque_R = o.max_torque_R;
    can_bus::note_command();   // refresh deadman
}
static void can_rx_update()  { can_bus::poll_rx(); }
static void safety_task()    { safety_update(); }

// --- task table: add a new module here (one line) ---
Task g_tasks[] = {
    { throttle_update,         10, 0 },   // 100 Hz
    { brake_update,            10, 0 },
    { steering_update,         10, 0 },
    { imu_update,              10, 0 },
    { wheel_speed_update,      10, 0 },
    { vehicle_speed_update,    10, 0 },   // 반드시 wheel_speed 다음
    { longitudinal_update,     10, 0 },
    { torque_vectoring_update, 10, 0 },
    { safety_task,             10, 0 },
    { can_rx_update,            5, 0 },   // 200 Hz drain
    { debug_update,           200, 0 },   // 5 Hz serial debug
};
const int G_TASK_COUNT = sizeof(g_tasks) / sizeof(g_tasks[0]);

void modules_init() {
    analogReadResolution(12);
    pinMode(board_pins::THROTTLE_ADC, INPUT);
    pinMode(board_pins::BRAKE_DIGITAL, INPUT);
    pinMode(board_pins::GEAR_ADC, INPUT);  // gear-ladder 모듈용 예약 입력
    for (int ch = 0; ch < WHEEL_COUNT; ++ch) wss_driver::begin(ch, PIN_WSS[ch]);
    imu_driver::begin();
    steering_encoder_driver::begin();
    can_bus::begin();
}
