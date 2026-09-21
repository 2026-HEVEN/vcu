#include <unity.h>
#include "safety_logic.h"
#include <cmath>

void test_shutdown_forces_halt(void) {
    SafetyInputs in{ false, true, true, false, true };   // shutdown_ok == false
    TEST_ASSERT_TRUE(SafetyState::Halt == safety_step(SafetyState::Drive, in));
}
void test_idle_to_ready_on_handshake(void) {
    SafetyInputs in{ true, true, true, false, true };
    TEST_ASSERT_TRUE(SafetyState::Ready == safety_step(SafetyState::Idle, in));
}
void test_ready_to_drive_on_start(void) {
    SafetyInputs in{ true, true, true, true, true };
    TEST_ASSERT_TRUE(SafetyState::Drive == safety_step(SafetyState::Ready, in));
}
void test_deadman_loss_halts(void) {
    SafetyInputs in{ true, true, false, false, true };
    TEST_ASSERT_TRUE(SafetyState::Halt == safety_step(SafetyState::Drive, in));
}
void test_invalid_throttle_signal_forces_halt(void) {
    SafetyInputs in{ true, true, true, true, false };
    TEST_ASSERT_TRUE(SafetyState::Halt == safety_step(SafetyState::Drive, in));
}

// --- motor_command_resolve: 좌·우 명령 확정 (M2) ---

// 전부 허용, Drive, 좌우 100 A. 각 테스트는 여기서 한 가지만 바꾼다.
static MotorCommandSnapshot mc_snapshot(void) {
    MotorCommandSnapshot s;
    s.seq = 42;
    s.published_ms = 1000;
    s.left_a = 100.0f;
    s.right_a = 100.0f;
    s.gear = Gear::Drive;
    s.safety_allow = true;
    s.throttle_signal_valid = true;
    s.propulsion_direction_armed = true;
    return s;
}

static MotorCommandGates mc_gates(void) {
    MotorCommandGates g;
    g.scheduler_alive = true;
    g.reconnect_inhibit = false;
    g.component_test_inhibit = false;
    g.snapshot_fresh = true;
    g.reconnect_ramp_scale = 1.0f;
    return g;
}

static MotorFrameCommand mc_resolve(const MotorCommandSnapshot &s,
                                    const MotorCommandGates &g) {
    return motor_command_resolve(s, g);
}

// 불허 결과는 항상 좌우 동일하게 완전 차단이어야 한다.
static void mc_assert_both_blocked(const MotorFrameCommand &cmd) {
    TEST_ASSERT_FALSE(cmd.normal_allow);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.left_a);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.right_a);
    TEST_ASSERT_FALSE(cmd.run_L);
    TEST_ASSERT_FALSE(cmd.run_R);
    TEST_ASSERT_EQUAL_INT(0, cmd.target_rpm_L);
    TEST_ASSERT_EQUAL_INT(0, cmd.target_rpm_R);
}

void test_mc_drive_commands_both_motors_forward(void) {
    const MotorFrameCommand cmd = mc_resolve(mc_snapshot(), mc_gates());
    TEST_ASSERT_TRUE(cmd.normal_allow);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cmd.left_a);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cmd.right_a);
    TEST_ASSERT_TRUE(cmd.run_L);
    TEST_ASSERT_TRUE(cmd.run_R);
    TEST_ASSERT_EQUAL_INT(4000, cmd.target_rpm_L);
    TEST_ASSERT_EQUAL_INT(4000, cmd.target_rpm_R);
}

void test_mc_reverse_commands_both_motors_backward(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.gear = Gear::Reverse;
    s.left_a = s.right_a = -100.0f;
    const MotorFrameCommand cmd = mc_resolve(s, mc_gates());
    TEST_ASSERT_TRUE(cmd.normal_allow);
    TEST_ASSERT_EQUAL_INT(-4000, cmd.target_rpm_L);
    TEST_ASSERT_EQUAL_INT(-4000, cmd.target_rpm_R);
}

void test_mc_regen_in_drive_targets_zero_rpm(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.left_a = -20.0f;
    s.right_a = -20.0f;
    s.brake_active = s.regen_allowed = s.forward_rotation = true;
    const MotorFrameCommand cmd = mc_resolve(s, mc_gates());
    TEST_ASSERT_TRUE(cmd.normal_allow);
    TEST_ASSERT_EQUAL_FLOAT(-20.0f, cmd.left_a);
    TEST_ASSERT_EQUAL_FLOAT(-20.0f, cmd.right_a);
    TEST_ASSERT_EQUAL_INT(0, cmd.target_rpm_L);
    TEST_ASSERT_EQUAL_INT(0, cmd.target_rpm_R);
}

void test_mc_regen_conditions_travel_with_snapshot(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.left_a = -10.0f; s.right_a = -15.0f;
    s.brake_active = s.regen_allowed = s.forward_rotation = true;
    for (int missing = 0; missing < 3; ++missing) {
        auto blocked = s;
        if (missing == 0) blocked.brake_active = false;
        if (missing == 1) blocked.regen_allowed = false;
        if (missing == 2) blocked.forward_rotation = false;
        const auto cmd = mc_resolve(blocked, mc_gates());
        TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.left_a);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.right_a);
        TEST_ASSERT_EQUAL_INT(0, cmd.target_rpm_L);
        TEST_ASSERT_EQUAL_INT(0, cmd.target_rpm_R);
    }
    // Replacing a snapshot changes current AND its interpretation together.
    s.gear = Gear::Reverse;
    s.brake_active = true;
    s.regen_allowed = s.forward_rotation = false;
    const auto reverse = mc_resolve(s, mc_gates());
    TEST_ASSERT_EQUAL_FLOAT(-10.0f, reverse.left_a);
    TEST_ASSERT_EQUAL_INT(-4000, reverse.target_rpm_L);
    TEST_ASSERT_EQUAL_INT(-4000, reverse.target_rpm_R);
    s.left_a = s.right_a = 10.0f; // Reverse regen is not supported.
    const auto invalid_reverse = mc_resolve(s, mc_gates());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, invalid_reverse.left_a);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, invalid_reverse.right_a);
}

void test_mc_brake_does_not_block_positive_throttle_drive(void) {
    auto s = mc_snapshot();
    s.brake_active = true;
    s.regen_allowed = false;
    const auto cmd = mc_resolve(s, mc_gates());
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cmd.left_a);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cmd.right_a);
    TEST_ASSERT_EQUAL_INT(4000, cmd.target_rpm_L);
}

void test_mc_safety_halt_blocks_both(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.safety_allow = false;
    mc_assert_both_blocked(mc_resolve(s, mc_gates()));
}

void test_mc_non_propulsion_gear_blocks_both(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.gear = Gear::Neutral;
    mc_assert_both_blocked(mc_resolve(s, mc_gates()));
    s.gear = Gear::Park;
    mc_assert_both_blocked(mc_resolve(s, mc_gates()));
}

void test_mc_stale_snapshot_blocks_both(void) {
    MotorCommandGates g = mc_gates();
    g.snapshot_fresh = false;
    mc_assert_both_blocked(mc_resolve(mc_snapshot(), g));
}

void test_mc_reconnect_ramp_scales_both_sides_equally(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.left_a = 300.0f;
    s.right_a = 200.0f;
    MotorCommandGates g = mc_gates();
    g.reconnect_ramp_scale = 0.5f;
    const MotorFrameCommand cmd = mc_resolve(s, g);
    TEST_ASSERT_EQUAL_FLOAT(150.0f, cmd.left_a);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cmd.right_a);
    // 램프는 비율이므로 좌우 비가 보존되어야 한다.
    TEST_ASSERT_EQUAL_FLOAT(s.left_a / s.right_a, cmd.left_a / cmd.right_a);
}

// M2 지적 ②의 회귀 방지. 좌우 전류 크기가 달라도 gear가 하나이므로
// 목표 rpm 부호는 절대 갈릴 수 없다.
void test_mc_asymmetric_current_never_splits_direction(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.left_a = 300.0f;
    s.right_a = 5.0f;
    MotorFrameCommand cmd = mc_resolve(s, mc_gates());
    TEST_ASSERT_EQUAL_INT(cmd.target_rpm_L, cmd.target_rpm_R);
    TEST_ASSERT_EQUAL_INT(4000, cmd.target_rpm_L);

    s.gear = Gear::Reverse;
    s.left_a = -300.0f;
    s.right_a = -5.0f;
    cmd = mc_resolve(s, mc_gates());
    TEST_ASSERT_EQUAL_INT(cmd.target_rpm_L, cmd.target_rpm_R);
    TEST_ASSERT_EQUAL_INT(-4000, cmd.target_rpm_L);
}

// NaN은 Clamped<> 도메인 타입을 그대로 통과한다. 한쪽만 NaN이어도
// 양쪽을 함께 차단해야 한다. 한쪽만 0으로 만들면 그것이 곧 좌우 비대칭이다.
void test_mc_non_finite_command_blocks_both_sides(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.left_a = std::nanf("");
    mc_assert_both_blocked(mc_resolve(s, mc_gates()));

    s = mc_snapshot();
    s.right_a = INFINITY;
    mc_assert_both_blocked(mc_resolve(s, mc_gates()));
}

void test_mc_direction_not_armed_blocks_both(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.propulsion_direction_armed = false;
    mc_assert_both_blocked(mc_resolve(s, mc_gates()));
}

void test_mc_invalid_throttle_signal_blocks_both(void) {
    MotorCommandSnapshot s = mc_snapshot();
    s.throttle_signal_valid = false;
    mc_assert_both_blocked(mc_resolve(s, mc_gates()));
}

void test_mc_core1_gates_block_both(void) {
    MotorCommandGates g = mc_gates();
    g.scheduler_alive = false;
    mc_assert_both_blocked(mc_resolve(mc_snapshot(), g));

    g = mc_gates();
    g.reconnect_inhibit = true;
    mc_assert_both_blocked(mc_resolve(mc_snapshot(), g));

    g = mc_gates();
    g.component_test_inhibit = true;
    mc_assert_both_blocked(mc_resolve(mc_snapshot(), g));
}

// 램프 스케일 자체가 오염돼도 명령을 키우거나 NaN을 내보내면 안 된다.
void test_mc_ramp_scale_is_clamped_and_nan_safe(void) {
    MotorCommandGates g = mc_gates();
    g.reconnect_ramp_scale = 2.0f;
    TEST_ASSERT_EQUAL_FLOAT(100.0f, mc_resolve(mc_snapshot(), g).left_a);

    g.reconnect_ramp_scale = -1.0f;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mc_resolve(mc_snapshot(), g).left_a);

    g.reconnect_ramp_scale = std::nanf("");
    const MotorFrameCommand cmd = mc_resolve(mc_snapshot(), g);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.left_a);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.right_a);
}


void setUp(void) {}
void test_snapshot_freshness_boundaries_and_rollover() {
    auto s=mc_snapshot();
    s.published_ms=1001;
    // Before-copy timestamp reproduces the reviewed false stale; use the later time.
    TEST_ASSERT_FALSE(motor_snapshot_fresh(s,1000,100));
    TEST_ASSERT_TRUE(motor_snapshot_fresh(s,1001,100));
    TEST_ASSERT_TRUE(motor_snapshot_fresh(s,1101,100));
    TEST_ASSERT_FALSE(motor_snapshot_fresh(s,1102,100));
    s.published_ms=0xFFFFFFF0u;
    TEST_ASSERT_TRUE(motor_snapshot_fresh(s,0x54u,100));
    TEST_ASSERT_FALSE(motor_snapshot_fresh(s,0x55u,100));
    s.seq=0;
    TEST_ASSERT_FALSE(motor_snapshot_fresh(s,0x54u,100));
}

void test_tx_failure_preserves_last_queued_command() {
    MotorTxSideDiagnostics d;
    motor_tx_record(d,MotorTxResult::Queued,100,4000,true,7,1000);
    motor_tx_record(d,MotorTxResult::Failed,0,0,false,8,1050);
    TEST_ASSERT_EQUAL_FLOAT(100,d.last_queued_a);
    TEST_ASSERT_TRUE(d.last_queued_running);
    TEST_ASSERT_EQUAL_UINT32(7,d.last_queued_seq);
    TEST_ASSERT_EQUAL_UINT32(1000,d.last_queued_ms);
    TEST_ASSERT_EQUAL_UINT32(1,d.failed_total);
    TEST_ASSERT_EQUAL_UINT(1,d.consecutive_failures);
    motor_tx_record(d,MotorTxResult::Failed,0,0,false,9,1100);
    motor_tx_record(d,MotorTxResult::Failed,0,0,false,10,1150);
    TEST_ASSERT_EQUAL_UINT(3,d.consecutive_failures);
    motor_tx_record(d,MotorTxResult::Queued,0,0,false,11,1200);
    TEST_ASSERT_EQUAL_UINT(0,d.consecutive_failures);
    TEST_ASSERT_EQUAL_UINT32(3,d.failed_total); // never lost on success
    TEST_ASSERT_EQUAL_FLOAT(0,d.last_queued_a);
    TEST_ASSERT_FALSE(d.last_queued_running);
}

void test_skipped_is_not_success_and_counters_saturate() {
    MotorTxSideDiagnostics d;
    motor_tx_record(d,MotorTxResult::Skipped,100,4000,true,1,0);
    TEST_ASSERT_FALSE(d.queued_valid);
    TEST_ASSERT_EQUAL_UINT32(0,d.failed_total);
    d.failed_total=UINT32_MAX; d.consecutive_failures=UINT32_MAX;
    motor_tx_record(d,MotorTxResult::Failed,100,4000,true,2,1);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX,d.failed_total);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX,d.consecutive_failures);
    motor_tx_record(d,MotorTxResult::Skipped,100,4000,true,3,2);
    TEST_ASSERT_FALSE(d.queued_valid);
    TEST_ASSERT_EQUAL_UINT(0,d.consecutive_failures);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX,d.failed_total);
}

void test_tx_pair_outcomes_are_independent() {
    MotorTxDiagnostics d;
    motor_tx_record(d.left,MotorTxResult::Queued,100,4000,true,4,10);
    motor_tx_record(d.right,MotorTxResult::Failed,100,4000,true,4,10);
    TEST_ASSERT_TRUE(d.left.queued_valid);
    TEST_ASSERT_FALSE(d.right.queued_valid);
    TEST_ASSERT_EQUAL_UINT32(0,d.left.failed_total);
    TEST_ASSERT_EQUAL_UINT32(1,d.right.failed_total);
}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_shutdown_forces_halt);
    RUN_TEST(test_idle_to_ready_on_handshake);
    RUN_TEST(test_ready_to_drive_on_start);
    RUN_TEST(test_deadman_loss_halts);
    RUN_TEST(test_invalid_throttle_signal_forces_halt);
    RUN_TEST(test_mc_drive_commands_both_motors_forward);
    RUN_TEST(test_mc_reverse_commands_both_motors_backward);
    RUN_TEST(test_mc_regen_in_drive_targets_zero_rpm);
    RUN_TEST(test_mc_regen_conditions_travel_with_snapshot);
    RUN_TEST(test_mc_brake_does_not_block_positive_throttle_drive);
    RUN_TEST(test_mc_safety_halt_blocks_both);
    RUN_TEST(test_mc_non_propulsion_gear_blocks_both);
    RUN_TEST(test_mc_stale_snapshot_blocks_both);
    RUN_TEST(test_mc_reconnect_ramp_scales_both_sides_equally);
    RUN_TEST(test_mc_asymmetric_current_never_splits_direction);
    RUN_TEST(test_mc_non_finite_command_blocks_both_sides);
    RUN_TEST(test_mc_direction_not_armed_blocks_both);
    RUN_TEST(test_mc_invalid_throttle_signal_blocks_both);
    RUN_TEST(test_mc_core1_gates_block_both);
    RUN_TEST(test_mc_ramp_scale_is_clamped_and_nan_safe);
    RUN_TEST(test_snapshot_freshness_boundaries_and_rollover);
    RUN_TEST(test_tx_failure_preserves_last_queued_command);
    RUN_TEST(test_skipped_is_not_success_and_counters_saturate);
    RUN_TEST(test_tx_pair_outcomes_are_independent);
    return UNITY_END();
}
