#pragma once

struct DriveSupervisorParams {
    float power_soft_limit_w;
    float drivetrain_efficiency;
    float motor_kt_nm_per_a;
    float paddock_current_per_motor_a;
    float paddock_speed_limit_mps;
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
    float vehicle_speed_mps;
};

struct DriveSupervisorOutput {
    float left_a = 0.0f;
    float right_a = 0.0f;
    float measured_bus_power_w = 0.0f;
    float estimated_input_power_w = 0.0f;
    float applied_scale = 0.0f;
    bool controller_blocked = false;
    bool power_limited = false;
    bool thermal_limited = false;
    bool paddock_limited = false;
};

DriveSupervisorOutput drive_supervisor_compute(
    const DriveSupervisorInput &in, const DriveSupervisorParams &params);
