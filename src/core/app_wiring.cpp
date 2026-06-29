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
#include "modules/longitudinal.h"
#include "modules/torque_vectoring.h"
#include <Arduino.h>

// [LOCKED] The ONLY translation unit that touches `state`. All update() wiring
// lives here; modules never see global state.
VehicleState state;

// --- per-module calibration (tune to the car) ---
namespace {
    constexpr int PIN_THROTTLE_ADC = 36;
    constexpr int PIN_BRAKE_ADC    = 39;
    constexpr int PIN_WSS          = 27;
    const WssCalib   WSS_CAL   { 45.0f };
    const SteerCalib STEER_CAL { 8192, 4096.0f, false };
    ImuFilterState   imu_state{};
    DriveMode        drive_mode = DriveMode::Normal;
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
    ImuOutput o = imu_compute(imu_driver::read(), imu_state);
    state.yaw_rate = o.yaw_rate; state.accel_x = o.accel_x; state.accel_y = o.accel_y;
}
static void wheel_speed_update() {
    state.wheel_speed = wheel_speed_compute(wss_driver::read(), WSS_CAL);
}
static void longitudinal_update() {
    state.total_torque = longitudinal_compute({
        state.throttle_pct, state.brake_pct, state.pack_soc, drive_mode });
}
static void torque_vectoring_update() {
    TVOutput o = tv_compute({
        state.total_torque, state.yaw_rate, state.steering_angle, state.wheel_speed });
    state.torque_L = o.torque_L; state.torque_R = o.torque_R;
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
    { longitudinal_update,     10, 0 },
    { torque_vectoring_update, 10, 0 },
    { safety_task,             10, 0 },
    { can_rx_update,            5, 0 },   // 200 Hz drain
    { debug_update,           200, 0 },   // 5 Hz serial debug
};
const int G_TASK_COUNT = sizeof(g_tasks) / sizeof(g_tasks[0]);

void modules_init() {
    analogReadResolution(12);
    wss_driver::begin(PIN_WSS);
    imu_driver::begin();
    steering_encoder_driver::begin();
    can_bus::begin();
}
