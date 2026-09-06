#pragma once

struct DriveSupervisorParams {
    float power_soft_limit_w;
    float drivetrain_efficiency;
    float motor_kt_nm_per_a;
    float paddock_current_zero_speed_per_motor_a;
    float paddock_current_high_speed_per_motor_a;
    float paddock_current_linear_end_speed_mps;
    float paddock_current_rise_time_s;
    float paddock_power_soft_limit_w;
    float paddock_controller_bus_current_limit_a;
    float paddock_pack_current_limit_a;
    float telemetry_temperature_valid_min_c;
    bool paddock_require_pack_data;
    float controller_derate_start_c;
    float controller_cutoff_c;
    float motor_derate_start_c;
    float motor_cutoff_c;
};

struct DriveSupervisorInput {
    float requested_left_a;
    float requested_right_a;
    bool controller_feedback_fresh;
    bool controller_fault;
    float bus_voltage_left_v;
    float bus_voltage_right_v;
    float bus_current_left_a;
    float bus_current_right_a;
    float phase_current_left_a;
    float phase_current_right_a;
    int motor_rpm_left;
    int motor_rpm_right;
    float controller_temp_left_c;
    float controller_temp_right_c;
    float motor_temp_left_c;
    float motor_temp_right_c;
    bool paddock_active;
    bool propulsion_requested;
    float control_dt_s;
    float vehicle_speed_mps;
    float paddock_speed_mps;
    bool pack_data_valid;
    float pack_current_a;
};

struct DriveSupervisorOutput {
    float left_a = 0.0f;
    float right_a = 0.0f;
    float measured_bus_power_w = 0.0f;
    float estimated_input_power_w = 0.0f;
    float paddock_current_limit_a = 0.0f;
    float applied_scale = 0.0f;
    bool controller_blocked = false;
    bool power_limited = false;
    bool thermal_limited = false;
    bool paddock_limited = false;
    bool paddock_sensor_blocked = false;
    bool paddock_current_limited = false;
    bool paddock_slew_limited = false;
};

struct DriveSupervisorState {
    float previous_left_a = 0.0f;
    float previous_right_a = 0.0f;
};

DriveSupervisorOutput drive_supervisor_compute(
    const DriveSupervisorInput &in, const DriveSupervisorParams &params,
    DriveSupervisorState &state);
