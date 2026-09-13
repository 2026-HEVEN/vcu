#include <unity.h>
#include <cmath>
#include "motor_command.h"

static MotorCommandParams params() { return MotorCommandParams{4000, 0}; }

// 전부 허용, Drive, 좌우 100 A. 각 테스트는 여기서 한 가지만 바꾼다.
static MotorCommandSnapshot nominal_snapshot() {
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

static MotorCommandGates nominal_gates() {
    MotorCommandGates g;
    g.scheduler_alive = true;
    g.reconnect_inhibit = false;
    g.component_test_inhibit = false;
    g.snapshot_fresh = true;
    g.reconnect_ramp_scale = 1.0f;
    return g;
}

static MotorFrameCommand resolve(const MotorCommandSnapshot &s,
                                 const MotorCommandGates &g) {
    return motor_command_resolve(s, g, params());
}

// 불허 결과는 항상 좌우 동일하게 완전 차단이어야 한다.
static void assert_both_blocked(const MotorFrameCommand &cmd) {
    TEST_ASSERT_FALSE(cmd.normal_allow);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.left_a);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.right_a);
    TEST_ASSERT_FALSE(cmd.run_L);
    TEST_ASSERT_FALSE(cmd.run_R);
    TEST_ASSERT_EQUAL_INT(0, cmd.target_rpm_L);
    TEST_ASSERT_EQUAL_INT(0, cmd.target_rpm_R);
}

static void test_drive_commands_both_motors_forward() {
    const MotorFrameCommand cmd = resolve(nominal_snapshot(), nominal_gates());
    TEST_ASSERT_TRUE(cmd.normal_allow);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cmd.left_a);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cmd.right_a);
    TEST_ASSERT_TRUE(cmd.run_L);
    TEST_ASSERT_TRUE(cmd.run_R);
    TEST_ASSERT_EQUAL_INT(4000, cmd.target_rpm_L);
    TEST_ASSERT_EQUAL_INT(4000, cmd.target_rpm_R);
}

static void test_reverse_commands_both_motors_backward() {
    MotorCommandSnapshot s = nominal_snapshot();
    s.gear = Gear::Reverse;
    const MotorFrameCommand cmd = resolve(s, nominal_gates());
    TEST_ASSERT_TRUE(cmd.normal_allow);
    TEST_ASSERT_EQUAL_INT(-4000, cmd.target_rpm_L);
    TEST_ASSERT_EQUAL_INT(-4000, cmd.target_rpm_R);
}

static void test_regen_in_drive_targets_zero_rpm() {
    MotorCommandSnapshot s = nominal_snapshot();
    s.left_a = -20.0f;
    s.right_a = -20.0f;
    const MotorFrameCommand cmd = resolve(s, nominal_gates());
    TEST_ASSERT_TRUE(cmd.normal_allow);
    TEST_ASSERT_EQUAL_INT(0, cmd.target_rpm_L);
    TEST_ASSERT_EQUAL_INT(0, cmd.target_rpm_R);
}

static void test_safety_halt_blocks_both() {
    MotorCommandSnapshot s = nominal_snapshot();
    s.safety_allow = false;
    assert_both_blocked(resolve(s, nominal_gates()));
}

static void test_non_propulsion_gear_blocks_both() {
    MotorCommandSnapshot s = nominal_snapshot();
    s.gear = Gear::Neutral;
    assert_both_blocked(resolve(s, nominal_gates()));
    s.gear = Gear::Park;
    assert_both_blocked(resolve(s, nominal_gates()));
}

static void test_stale_snapshot_blocks_both() {
    MotorCommandGates g = nominal_gates();
    g.snapshot_fresh = false;
    assert_both_blocked(resolve(nominal_snapshot(), g));
}

static void test_reconnect_ramp_scales_both_sides_equally() {
    MotorCommandSnapshot s = nominal_snapshot();
    s.left_a = 300.0f;
    s.right_a = 200.0f;
    MotorCommandGates g = nominal_gates();
    g.reconnect_ramp_scale = 0.5f;
    const MotorFrameCommand cmd = resolve(s, g);
    TEST_ASSERT_EQUAL_FLOAT(150.0f, cmd.left_a);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, cmd.right_a);
    // 램프는 비율이므로 좌우 비가 보존되어야 한다.
    TEST_ASSERT_EQUAL_FLOAT(s.left_a / s.right_a, cmd.left_a / cmd.right_a);
}

// M2 지적 ②의 회귀 방지. 좌우 전류 크기가 달라도 기어가 하나이므로
// 목표 rpm 부호는 절대 갈릴 수 없다.
static void test_asymmetric_current_never_splits_direction() {
    MotorCommandSnapshot s = nominal_snapshot();
    s.left_a = 300.0f;
    s.right_a = 5.0f;
    MotorFrameCommand cmd = resolve(s, nominal_gates());
    TEST_ASSERT_EQUAL_INT(cmd.target_rpm_L, cmd.target_rpm_R);
    TEST_ASSERT_EQUAL_INT(4000, cmd.target_rpm_L);

    s.gear = Gear::Reverse;
    cmd = resolve(s, nominal_gates());
    TEST_ASSERT_EQUAL_INT(cmd.target_rpm_L, cmd.target_rpm_R);
    TEST_ASSERT_EQUAL_INT(-4000, cmd.target_rpm_L);
}

// NaN은 Clamped<> 도메인 타입을 그대로 통과한다. 한쪽만 NaN이어도
// 양쪽을 함께 차단해야 한다. 한쪽만 0으로 만들면 그것이 곧 좌우 비대칭이다.
static void test_non_finite_command_blocks_both_sides() {
    MotorCommandSnapshot s = nominal_snapshot();
    s.left_a = std::nanf("");
    assert_both_blocked(resolve(s, nominal_gates()));

    s = nominal_snapshot();
    s.right_a = INFINITY;
    assert_both_blocked(resolve(s, nominal_gates()));
}

static void test_direction_not_armed_blocks_both() {
    MotorCommandSnapshot s = nominal_snapshot();
    s.propulsion_direction_armed = false;
    assert_both_blocked(resolve(s, nominal_gates()));
}

static void test_invalid_throttle_signal_blocks_both() {
    MotorCommandSnapshot s = nominal_snapshot();
    s.throttle_signal_valid = false;
    assert_both_blocked(resolve(s, nominal_gates()));
}

static void test_core1_gates_block_both() {
    MotorCommandGates g = nominal_gates();
    g.scheduler_alive = false;
    assert_both_blocked(resolve(nominal_snapshot(), g));

    g = nominal_gates();
    g.reconnect_inhibit = true;
    assert_both_blocked(resolve(nominal_snapshot(), g));

    g = nominal_gates();
    g.component_test_inhibit = true;
    assert_both_blocked(resolve(nominal_snapshot(), g));
}

// 램프 스케일 자체가 오염돼도 명령을 키우거나 NaN을 내보내면 안 된다.
static void test_ramp_scale_is_clamped_and_nan_safe() {
    MotorCommandGates g = nominal_gates();
    g.reconnect_ramp_scale = 2.0f;
    TEST_ASSERT_EQUAL_FLOAT(100.0f, resolve(nominal_snapshot(), g).left_a);

    g.reconnect_ramp_scale = -1.0f;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, resolve(nominal_snapshot(), g).left_a);

    g.reconnect_ramp_scale = std::nanf("");
    const MotorFrameCommand cmd = resolve(nominal_snapshot(), g);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.left_a);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cmd.right_a);
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_drive_commands_both_motors_forward);
    RUN_TEST(test_reverse_commands_both_motors_backward);
    RUN_TEST(test_regen_in_drive_targets_zero_rpm);
    RUN_TEST(test_safety_halt_blocks_both);
    RUN_TEST(test_non_propulsion_gear_blocks_both);
    RUN_TEST(test_stale_snapshot_blocks_both);
    RUN_TEST(test_reconnect_ramp_scales_both_sides_equally);
    RUN_TEST(test_asymmetric_current_never_splits_direction);
    RUN_TEST(test_non_finite_command_blocks_both_sides);
    RUN_TEST(test_direction_not_armed_blocks_both);
    RUN_TEST(test_invalid_throttle_signal_blocks_both);
    RUN_TEST(test_core1_gates_block_both);
    RUN_TEST(test_ramp_scale_is_clamped_and_nan_safe);
    return UNITY_END();
}
