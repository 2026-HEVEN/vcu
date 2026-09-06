#include <unity.h>
#include "modules/realcar_calibration.h"
#include "modules/vehicle_speed.h"
#include "modules/tv/tv_config.h"

void test_all_four_wss_channels_use_confirmed_24_ppr() {
    TEST_ASSERT_EQUAL_FLOAT(24.0f, realcar_cal::confirmed::WSS_PULSES_PER_WHEEL_REV_FL);
    TEST_ASSERT_EQUAL_FLOAT(24.0f, realcar_cal::confirmed::WSS_PULSES_PER_WHEEL_REV_FR);
    TEST_ASSERT_EQUAL_FLOAT(24.0f, realcar_cal::confirmed::WSS_PULSES_PER_WHEEL_REV_RL);
    TEST_ASSERT_EQUAL_FLOAT(24.0f, realcar_cal::confirmed::WSS_PULSES_PER_WHEEL_REV_RR);
}

void test_vehicle_speed_defaults_share_realcar_profile() {
    const VehicleSpeedCalib c{};
    TEST_ASSERT_FLOAT_WITHIN(0.0001f,
        realcar_cal::provisional::WHEEL_SPEED_ROLLING_RADIUS_M, c.tire_radius_m);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f,
        realcar_cal::provisional::FRONT_TRACK_M, c.track_m);
}

void test_tv_defaults_share_realcar_profile() {
    TEST_ASSERT_FLOAT_WITHIN(0.0001f,
        realcar_cal::provisional::VEHICLE_MASS_WITH_DRIVER_KG, TV_PARAMS.mass_kg);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f,
        realcar_cal::provisional::WHEELBASE_M, TV_PARAMS.wheelbase_m);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f,
        realcar_cal::provisional::REAR_TRACK_M, TV_PARAMS.track_m);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f,
        realcar_cal::provisional::TV_FORCE_RADIUS_M, TV_PARAMS.tire_radius_m);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f,
        realcar_cal::confirmed::GEAR_RATIO, TV_PARAMS.gear_ratio);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f,
        realcar_cal::confirmed::MOTOR_KT_NM_PER_A, TV_PARAMS.motor_kt_nm_per_a);
}

void test_production_default_keeps_tv_master_off() {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, TV_PARAMS.kp);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, TV_PARAMS.ki);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, TV_PARAMS.kd);
}

void test_dual_motor_bringup_requires_both_controllers() {
    TEST_ASSERT_TRUE(realcar_cal::bringup::REQUIRE_BOTH_MOTOR_CONTROLLERS);
}

void test_bringup_drive_phase_current_ceiling_is_500_a_per_motor() {
    TEST_ASSERT_EQUAL_FLOAT(
        500.0f, realcar_cal::bringup::DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A);
    TEST_ASSERT_EQUAL_FLOAT(
        realcar_cal::bringup::DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A,
        TV_PARAMS.motor_current_max_a);
    TEST_ASSERT_TRUE(
        realcar_cal::bringup::DRIVE_PHASE_CURRENT_EFF_PER_MOTOR_A <=
        realcar_cal::bringup::DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A);
    TEST_ASSERT_EQUAL_FLOAT(
        150.0f, realcar_cal::bringup::COMPONENT_TEST_CURRENT_MAX_PER_MOTOR_A);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f,
        realcar_cal::bringup::DRIVE_CURRENT_RISE_TIME_S);
}

void test_unverified_inputs_are_fail_closed() {
    TEST_ASSERT_TRUE(realcar_cal::bringup::GEAR_SELECTOR_INSTALLED);
    TEST_ASSERT_FALSE(realcar_cal::bringup::REGEN_HARDWARE_VALIDATED);
    TEST_ASSERT_FALSE(realcar_cal::bringup::ENABLE_DRIVE_POWER_LIMIT);
    TEST_ASSERT_FALSE(realcar_cal::bringup::PADDOCK_CURRENT_CALIBRATED);
    TEST_ASSERT_EQUAL_FLOAT(9000.0f,
        realcar_cal::bringup::DRIVE_POWER_SOFT_LIMIT_W);
}

void test_can_rx_queue_has_burst_margin_for_debug_logging() {
    TEST_ASSERT_EQUAL_UINT(32U, realcar_cal::bringup::CAN_RX_QUEUE_LENGTH);
}

void test_rehandshake_timeout_and_recovery_timing_are_configured() {
    TEST_ASSERT_TRUE(realcar_cal::bringup::MOTOR_COMMAND_PERIOD_MS > 0U);
    TEST_ASSERT_TRUE(
        realcar_cal::bringup::CONTROLLER_FEEDBACK_STALE_MS <
        realcar_cal::bringup::CONTROLLER_REHANDSHAKE_TIMEOUT_MS);
    TEST_ASSERT_TRUE(
        realcar_cal::bringup::CONTROLLER_REHANDSHAKE_TIMEOUT_MS >= 500U);
    TEST_ASSERT_EQUAL_UINT(
        300U, realcar_cal::bringup::COMPONENT_TEST_RELEASE_HOLD_MS);
    TEST_ASSERT_EQUAL_UINT(
        (realcar_cal::bringup::COMPONENT_TEST_RELEASE_HOLD_MS +
         realcar_cal::bringup::MOTOR_COMMAND_PERIOD_MS - 1U) /
            realcar_cal::bringup::MOTOR_COMMAND_PERIOD_MS,
        realcar_cal::bringup::COMPONENT_TEST_RELEASE_TICKS);
    TEST_ASSERT_EQUAL_UINT(
        1000U, realcar_cal::bringup::MOTOR_RECONNECT_RAMP_MS);
}

void test_throttle_signal_and_zero_percent_thresholds() {
    TEST_ASSERT_EQUAL_UINT(400U,
        realcar_cal::bringup::THROTTLE_SIGNAL_VALID_MIN_ADC);
    TEST_ASSERT_EQUAL_FLOAT(500.0f,
        realcar_cal::provisional::THROTTLE_RAW_MIN);
    TEST_ASSERT_EQUAL_FLOAT(3000.0f,
        realcar_cal::provisional::THROTTLE_RAW_MAX);
    TEST_ASSERT_TRUE(realcar_cal::bringup::THROTTLE_SIGNAL_VALID_MIN_ADC <
        realcar_cal::provisional::THROTTLE_RAW_MIN);
}

void test_paddock_speed_current_profile_is_bounded() {
    TEST_ASSERT_TRUE(
        realcar_cal::bringup::PADDOCK_CURRENT_ZERO_SPEED_PER_MOTOR_A <=
        realcar_cal::bringup::DRIVE_PHASE_CURRENT_MAX_PER_MOTOR_A);
    TEST_ASSERT_TRUE(
        realcar_cal::bringup::PADDOCK_CURRENT_HIGH_SPEED_PER_MOTOR_A <
        realcar_cal::bringup::PADDOCK_CURRENT_ZERO_SPEED_PER_MOTOR_A);
    TEST_ASSERT_TRUE(
        realcar_cal::bringup::PADDOCK_CURRENT_HIGH_SPEED_PER_MOTOR_A <=
        realcar_cal::confirmed::MOTOR_CONTINUOUS_CURRENT_MAX_A);
    TEST_ASSERT_TRUE(
        realcar_cal::bringup::PADDOCK_CURRENT_LINEAR_END_SPEED_MPS > 0.0f);
    TEST_ASSERT_TRUE(
        realcar_cal::bringup::PADDOCK_POWER_SOFT_LIMIT_W > 0.0f);
    TEST_ASSERT_TRUE(
        realcar_cal::bringup::PADDOCK_PACK_CURRENT_LIMIT_A < 157.0f);
    TEST_ASSERT_TRUE(realcar_cal::bringup::PADDOCK_REQUIRE_PACK_DATA);
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_all_four_wss_channels_use_confirmed_24_ppr);
    RUN_TEST(test_vehicle_speed_defaults_share_realcar_profile);
    RUN_TEST(test_tv_defaults_share_realcar_profile);
    RUN_TEST(test_production_default_keeps_tv_master_off);
    RUN_TEST(test_dual_motor_bringup_requires_both_controllers);
    RUN_TEST(test_bringup_drive_phase_current_ceiling_is_500_a_per_motor);
    RUN_TEST(test_unverified_inputs_are_fail_closed);
    RUN_TEST(test_can_rx_queue_has_burst_margin_for_debug_logging);
    RUN_TEST(test_rehandshake_timeout_and_recovery_timing_are_configured);
    RUN_TEST(test_throttle_signal_and_zero_percent_thresholds);
    RUN_TEST(test_paddock_speed_current_profile_is_bounded);
    return UNITY_END();
}
