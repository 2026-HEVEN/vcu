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
#include "modules/throttle.h"
#include "modules/brake.h"
#include "modules/steering.h"
#include "modules/imu.h"
#include "modules/wheel_speed.h"
#include "modules/vehicle_speed.h"
#include "modules/longitudinal.h"
#include "modules/torque_vectoring.h"
#include <Arduino.h>

// [LOCKED] The ONLY translation unit that touches `state`. All update() wiring
// lives here; modules never see global state.
VehicleState state;

// --- per-module calibration (tune to the car) ---
namespace {
    // --- GPIO 배정: 노션 「하네스 설계 (v4)」 기준 ---
    //     https://app.notion.com/p/399913e532e6810fa2ffeac09c02f0f9
    //     핀을 바꾸려면 그 문서를 먼저 고치고 여기를 맞출 것 (문서가 기준).
    constexpr int PIN_THROTTLE_ADC = 33;   // v4: 구 D35 → D33 이동
    constexpr int PIN_BRAKE_ADC    = 32;   // v4: 12V→3.3V 분압
    // WSS 4채널 — 전부 input-only 핀. LM393 오픈컬렉터라 외부 3.3V 풀업 필수.
    constexpr int PIN_WSS[WHEEL_COUNT] = {
        36,   // WHEEL_FL
        39,   // WHEEL_FR
        34,   // WHEEL_RL
        35,   // WHEEL_RR
    };

    // TODO(배선팀): 자석 수 N이 확정되면 교체. 노션 「WSS 자석 수 계산기」의
    //   시나리오가 아직 전부 "예시" 상태라 기존 값 45를 4륜에 그대로 둔다.
    //   림 부착 방식이라 바퀴마다 자석 수가 다를 수 있으므로 채널별로 분리해 둠.
    const WssCalib WSS_CAL[WHEEL_COUNT] = {
        { 45.0f }, { 45.0f }, { 45.0f }, { 45.0f },
    };
    const VehicleSpeedCalib VSPEED_CAL{};   // 기본값: 구름반경 0.165m, 윤거 1.20m
    VehicleSpeedState vspeed_state{};
    const SteerCalib STEER_CAL { 8192, 4096.0f, false };
    TVYawState       tv_yaw_state{};       // yaw 제어기 이력 (전역상태 아님, 여기서만 보유)
    DriveMode        drive_mode = DriveMode::Normal;
    constexpr float  TV_DT_S = 0.01f;      // torque_vectoring_update 주기 (10ms task)
    constexpr float  WHEEL_SPEED_DT_S = 0.01f;  // wheel/vehicle speed task 주기 (10ms)
}

static void throttle_update() {
    state.throttle_pct = throttle_compute({ analogRead(PIN_THROTTLE_ADC) });
}
static void brake_update() {
    BrakeOutput o = brake_compute({ analogRead(PIN_BRAKE_ADC) });
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
        state.wheel_speed[ch] = wheel_speed_compute(wss_driver::read(ch), WSS_CAL[ch]);
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
    TVOutput o = tv_compute({
        state.total_torque, state.yaw_rate, state.steering_angle,
        // 전륜 신호를 못 믿으면 차속 0 → reference stage의 저속 컷오프에 걸려 TV가 꺼진다.
        state.vehicle_speed_valid ? state.vehicle_speed_mps : 0.0f,
        state.accel_x, state.accel_y, TV_DT_S }, tv_yaw_state);
    state.torque_L = o.torque_L; state.torque_R = o.torque_R;
    // 중간신호 관측용 복사 (debug_monitor / Cluster에서 튜닝에 사용)
    state.desired_yaw_rate = o.desired_yaw_rate;
    state.yaw_moment       = o.yaw_moment;
    state.fz_L = o.fz_L; state.fz_R = o.fz_R;
    state.max_torque_L = o.max_torque_L; state.max_torque_R = o.max_torque_R;
    can_bus::note_command();   // refresh deadman
}
static void can_rx_update()  { can_bus::poll_rx(); }
static void wheel_speed_can_tx_update() { can_bus::send_wheel_speeds(); }
static void vehicle_speed_can_tx_update() { can_bus::send_vehicle_speed(); }
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
    { wheel_speed_can_tx_update, 50, 0 }, // 20 Hz VCU -> Cluster WSS telemetry
    { vehicle_speed_can_tx_update, 50, 0 }, // 20 Hz VCU -> TMA-1 single speed telemetry
    { debug_update,           200, 0 },   // 5 Hz serial debug
};
const int G_TASK_COUNT = sizeof(g_tasks) / sizeof(g_tasks[0]);

void modules_init() {
    analogReadResolution(12);
    for (int ch = 0; ch < WHEEL_COUNT; ++ch) wss_driver::begin(ch, PIN_WSS[ch]);
    imu_driver::begin();
    steering_encoder_driver::begin();
    can_bus::begin();
}
