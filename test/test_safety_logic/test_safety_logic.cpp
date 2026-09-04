#include <unity.h>
#include "safety_logic.h"

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

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_shutdown_forces_halt);
    RUN_TEST(test_idle_to_ready_on_handshake);
    RUN_TEST(test_ready_to_drive_on_start);
    RUN_TEST(test_deadman_loss_halts);
    RUN_TEST(test_invalid_throttle_signal_forces_halt);
    return UNITY_END();
}
