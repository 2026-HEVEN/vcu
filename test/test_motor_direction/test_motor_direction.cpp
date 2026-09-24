#include <unity.h>
#include <cmath>
#include <initializer_list>
#include "modules/motor_direction.h"
#include "modules/longitudinal.h"
#include "modules/torque_vectoring.h"
#include "modules/drive_supervisor.h"
#include "can_protocol.h"

void setUp() {}
void tearDown() {}

void test_demand_sign_and_gears() {
    TEST_ASSERT_EQUAL_FLOAT(300, directional_current(300, Gear::Drive, true));
    TEST_ASSERT_EQUAL_FLOAT(-20, directional_current(-20, Gear::Drive, true));
    TEST_ASSERT_EQUAL_FLOAT(-300, directional_current(300, Gear::Reverse, true));
    TEST_ASSERT_EQUAL_FLOAT(0, directional_current(-20, Gear::Reverse, true));
    for (Gear gear : {Gear::Neutral, Gear::Park, static_cast<Gear>(255)}) {
        TEST_ASSERT_EQUAL_FLOAT(0, directional_current(300, gear, true));
        TEST_ASSERT_EQUAL_FLOAT(0, directional_current(-20, gear, true));
    }
    TEST_ASSERT_EQUAL_FLOAT(0, directional_current(-20, Gear::Drive, false));
    TEST_ASSERT_EQUAL_FLOAT(0, directional_current(NAN, Gear::Drive, true));
}

void test_wire_direction_combinations() {
    const auto drive = motor_direction_command(10, Gear::Drive, true, false, true);
    const auto regen = motor_direction_command(-10, Gear::Drive, true, true, true);
    const auto reverse = motor_direction_command(-10, Gear::Reverse, true, false, false);
    TEST_ASSERT_EQUAL_FLOAT(10, drive.current_a);
    TEST_ASSERT_EQUAL_INT(4000, drive.target_rpm);
    TEST_ASSERT_EQUAL_FLOAT(-10, regen.current_a);
    TEST_ASSERT_EQUAL_INT(0, regen.target_rpm);
    TEST_ASSERT_EQUAL_FLOAT(-10, reverse.current_a);
    TEST_ASSERT_EQUAL_INT(-4000, reverse.target_rpm);
    uint8_t data[8];
    encode_motor_control(regen.current_a, regen.target_rpm, regen.running, 1, data);
    TEST_ASSERT_EQUAL_UINT16(31900, data[0] | (data[1] << 8));
    TEST_ASSERT_EQUAL_UINT16(32000, data[2] | (data[3] << 8));
    TEST_ASSERT_EQUAL_UINT8(1, data[4]); // running torque mode, not speed mode
}

void test_regen_permission_and_motion_guards() {
    TEST_ASSERT_EQUAL_FLOAT(0, motor_direction_command(-10, Gear::Drive, true, false, true).current_a);
    TEST_ASSERT_EQUAL_FLOAT(0, motor_direction_command(-10, Gear::Drive, true, true, false).current_a);
    TEST_ASSERT_FALSE(motor_direction_command(-10, Gear::Drive, false, true, true).running);
    TEST_ASSERT_FALSE(motor_direction_command(-10, Gear::Neutral, true, true, true).running);
    TEST_ASSERT_FALSE(motor_direction_command(NAN, Gear::Drive, true, true, true).running);
}

void test_forward_rpm_threshold_and_reverse_drive() {
    TEST_ASSERT_TRUE(regen_forward_rotation_ok(51, -51, true, 50));
    TEST_ASSERT_FALSE(regen_forward_rotation_ok(50, -51, true, 50));
    TEST_ASSERT_FALSE(regen_forward_rotation_ok(51, -50, true, 50));
    TEST_ASSERT_FALSE(regen_forward_rotation_ok(51, -51, false, 50));
    TEST_ASSERT_FALSE(regen_forward_rotation_ok(-51, 51, true, 50));
    TEST_ASSERT_EQUAL_FLOAT(10, motor_direction_command(10, Gear::Drive, true, false, true).current_a);
    TEST_ASSERT_EQUAL_FLOAT(-10, motor_direction_command(-10, Gear::Reverse, true, false, false).current_a);
    TEST_ASSERT_EQUAL_FLOAT(0, motor_direction_command(10, Gear::Reverse, true, true, false).current_a);
    const float drive = longitudinal_compute({100, 0.5f, DriveMode::Normal, false});
    TEST_ASSERT_EQUAL_FLOAT(-drive, directional_current(drive, Gear::Reverse, true));
}

void test_forward_regen_survives_longitudinal_tv_and_wire() {
    const float demand = longitudinal_compute({0, 0.5f, DriveMode::Normal, true});
    TEST_ASSERT_EQUAL_FLOAT(-20, demand);
    const float signed_demand = directional_current(demand, Gear::Drive, true);
    TVInput input{};
    input.total_torque = Ampere{signed_demand};
    input.dt = Seconds{0.01f};
    TVYawState state{};
    const auto tv = tv_compute(input, state); // TV OFF: identical split
    DriveSupervisorInput supervisor_in{};
    supervisor_in.requested_left_a = (float)tv.torque_L;
    supervisor_in.requested_right_a = (float)tv.torque_R;
    supervisor_in.controller_feedback_fresh = true;
    supervisor_in.control_dt_s = 0.01f;
    DriveSupervisorParams params{};
    params.controller_derate_start_c = params.motor_derate_start_c = 70;
    params.controller_cutoff_c = params.motor_cutoff_c = 90;
    DriveSupervisorState supervisor_state{};
    const auto supervised = drive_supervisor_compute(supervisor_in, params, supervisor_state);
    for (float current : {supervised.left_a, supervised.right_a}) {
        const auto wire = motor_direction_command(current, Gear::Drive, true, true, true);
        TEST_ASSERT_EQUAL_FLOAT(-10, wire.current_a);
        TEST_ASSERT_EQUAL_INT(0, wire.target_rpm);
    }
}

void test_regen_to_overlap_drive_restarts_ramp() {
    DriveSupervisorParams params{};
    params.drive_current_max_per_motor_a = 500;
    params.drive_current_rise_time_s = 0.5f;
    params.controller_derate_start_c = params.motor_derate_start_c = 70;
    params.controller_cutoff_c = params.motor_cutoff_c = 90;
    for (Gear gear : {Gear::Drive, Gear::Reverse}) {
        DriveSupervisorState state{};
        DriveSupervisorInput in{};
        in.controller_feedback_fresh = true;
        in.control_dt_s = 0.01f;
        in.requested_left_a = in.requested_right_a = -10;
        drive_supervisor_compute(in, params, state); // regen resets launch history
        const float demand = longitudinal_compute({100, 0.5f, DriveMode::Normal, true});
        in.requested_left_a = in.requested_right_a = directional_current(demand, gear, true) / 2;
        in.propulsion_requested = true; // overlap still counts as propulsion
        const auto out = drive_supervisor_compute(in, params, state);
        const auto cmd = motor_direction_command(out.left_a, gear, true, false, true);
        TEST_ASSERT_TRUE(out.drive_slew_limited);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, gear == Gear::Drive ? 10 : -10, cmd.current_a);
        TEST_ASSERT_EQUAL_INT(gear == Gear::Drive ? 4000 : -4000, cmd.target_rpm);
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_demand_sign_and_gears);
    RUN_TEST(test_wire_direction_combinations);
    RUN_TEST(test_regen_permission_and_motion_guards);
    RUN_TEST(test_forward_rpm_threshold_and_reverse_drive);
    RUN_TEST(test_forward_regen_survives_longitudinal_tv_and_wire);
    RUN_TEST(test_regen_to_overlap_drive_restarts_ramp);
    return UNITY_END();
}
