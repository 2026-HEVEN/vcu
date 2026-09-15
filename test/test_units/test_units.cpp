// 차원 타입(Qty) 계약.
// Clamped는 "경계가 있는 액추에이터 명령/센서 값"이고, Qty는 "계산 중인 물리량"이다.
// 둘의 결정적 차이는 Qty가 자르지 않는다는 것 — dt가 NaN이거나 ax가 50 g로
// 들어오면 각 stage의 isfinite 가드까지 그대로 전달돼야 한다. 경계값으로
// 둔갑시키면 진단이 아니라 은폐다.
#include <unity.h>
#include <type_traits>
#include <cmath>
#include "types.h"

void test_raw_float_does_not_implicitly_become_a_quantity(void) {
    TEST_ASSERT_FALSE((std::is_convertible<float, Mps>::value));
    TEST_ASSERT_FALSE((std::is_convertible<float, NewtonMetre>::value));
    TEST_ASSERT_FALSE((std::is_convertible<Mps, float>::value));
}

void test_different_dimensions_do_not_interconvert(void) {
    // deg/s 를 N.m 자리에, m/s 를 s 자리에 넣는 사고를 막는 것이 목적이다.
    TEST_ASSERT_FALSE((std::is_convertible<Mps, DegPerSec>::value));
    TEST_ASSERT_FALSE((std::is_convertible<DegPerSec, NewtonMetre>::value));
    TEST_ASSERT_FALSE((std::is_convertible<Seconds, Mps>::value));
    TEST_ASSERT_FALSE((std::is_convertible<Newton, Ampere>::value));
    TEST_ASSERT_FALSE((std::is_convertible<GForce, Mps>::value));
}

void test_explicit_construction_round_trips(void) {
    TEST_ASSERT_TRUE((std::is_constructible<Mps, float>::value));
    TEST_ASSERT_EQUAL_FLOAT(13.5f, (float)Mps{13.5f});
}

void test_quantities_do_not_clamp(void) {
    // Amp은 500에서 자르지만 Ampere는 물리량이라 자르지 않는다.
    TEST_ASSERT_EQUAL_FLOAT(500.0f, (float)Amp{4800.0f});
    TEST_ASSERT_EQUAL_FLOAT(4800.0f, (float)Ampere{4800.0f});
    TEST_ASSERT_EQUAL_FLOAT(-1000.0f, (float)Ampere{-1000.0f});
}

void test_quantities_pass_nan_through_to_the_guards(void) {
    const float nan_value = NAN;
    TEST_ASSERT_TRUE(std::isnan((float)Seconds{nan_value}));
    TEST_ASSERT_TRUE(std::isnan((float)GForce{nan_value}));
}

void test_default_is_zero_and_value_initialises(void) {
    TEST_ASSERT_TRUE((std::is_default_constructible<Newton>::value));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)Newton{});
}

void test_per_motor_command_and_physical_current_are_distinct_types(void) {
    // Amp = 모터 1개 명령(±500, 클램프). Ampere = 물리량(좌우 합 ±1000 등).
    TEST_ASSERT_FALSE((std::is_convertible<Ampere, Amp>::value));
    TEST_ASSERT_FALSE((std::is_convertible<Amp, Ampere>::value));
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_raw_float_does_not_implicitly_become_a_quantity);
    RUN_TEST(test_different_dimensions_do_not_interconvert);
    RUN_TEST(test_explicit_construction_round_trips);
    RUN_TEST(test_quantities_do_not_clamp);
    RUN_TEST(test_quantities_pass_nan_through_to_the_guards);
    RUN_TEST(test_default_is_zero_and_value_initialises);
    RUN_TEST(test_per_motor_command_and_physical_current_are_distinct_types);
    return UNITY_END();
}
