#include <unity.h>
#include <type_traits>
#include "types.h"
#include "state.h"

// ── 차원 계약 ────────────────────────────────────────────────────────────
// Clamped는 범위만 강제하고 차원은 이름에만 있었다. float를 경유하면 Amp와
// Percent가 조용히 서로 대입됐고, 그래서 #15(토크 명령을 Percent로 선언)가
// 났다. 경계에서 명시적 형변환을 강제해 "이 차원의 값을 의도했다"를 코드에
// 남긴다. 중간 계산은 생 float로 자유롭게 둔다.
void test_float_does_not_implicitly_become_a_domain_type(void) {
    TEST_ASSERT_FALSE((std::is_convertible<float, Amp>::value));
    TEST_ASSERT_FALSE((std::is_convertible<float, Percent>::value));
}
void test_domain_type_does_not_implicitly_become_float(void) {
    TEST_ASSERT_FALSE((std::is_convertible<Amp, float>::value));
}
void test_domain_types_do_not_launder_through_float(void) {
    // Amp -> float -> Percent 경로가 막혀야 한다. 이게 #15의 사고 유형이다.
    TEST_ASSERT_FALSE((std::is_convertible<Amp, Percent>::value));
    TEST_ASSERT_FALSE((std::is_convertible<Percent, Amp>::value));
    TEST_ASSERT_FALSE((std::is_convertible<Unit, Amp>::value));
}
void test_explicit_construction_and_cast_still_work(void) {
    TEST_ASSERT_TRUE((std::is_constructible<Amp, float>::value));
    Amp a{250.0f};
    TEST_ASSERT_EQUAL_FLOAT(250.0f, (float)a);
}
void test_raw_float_cannot_be_assigned_into_a_domain_type(void) {
    // 생성만 막고 대입을 열어두면 `float mz = ...; Amp a; a = mz;` 로 그대로
    // 샌다. 이게 막으려던 사고 경로 자체이므로 대입도 Amp(x)를 요구한다.
    TEST_ASSERT_FALSE((std::is_assignable<Amp &, float>::value));
    TEST_ASSERT_FALSE((std::is_assignable<Unit &, float>::value));
    TEST_ASSERT_TRUE((std::is_assignable<Amp &, Amp>::value));
}
void test_value_initialisation_survives(void) {
    // 기본 생성자까지 explicit으로 막으면 `TVAllocOutput a{};` 같은 값초기화가
    // 전부 깨진다. 기본 생성자는 암묵으로 남겨 aggregate 사용성을 지킨다.
    TEST_ASSERT_TRUE((std::is_default_constructible<Amp>::value));
    Amp a{};
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)a);
}

void test_clamps_above_max(void) { Percent p{150.0f}; TEST_ASSERT_EQUAL_FLOAT(100.0f, (float)p); }
void test_clamps_below_min(void) { Percent p{-250.0f}; TEST_ASSERT_EQUAL_FLOAT(-100.0f, (float)p); }
void test_passes_in_range(void) { Percent p{42.0f}; TEST_ASSERT_EQUAL_FLOAT(42.0f, (float)p); }
void test_assignment_clamps(void) { Unit u{}; u = Unit{5.0f}; TEST_ASSERT_EQUAL_FLOAT(1.0f, (float)u); }
void test_default_is_zero(void) { Rpm r; TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)r); }
void test_amp_clamps_above_max(void) { Amp a{600.0f}; TEST_ASSERT_EQUAL_FLOAT(500.0f, (float)a); }
void test_amp_clamps_below_min(void) { Amp a{-600.0f}; TEST_ASSERT_EQUAL_FLOAT(-500.0f, (float)a); }
void test_amp_allows_continuous_limit(void) { Amp a{103.0f}; TEST_ASSERT_EQUAL_FLOAT(103.0f, (float)a); }
void test_amp_allows_peak_limit(void) { Amp a{500.0f}; TEST_ASSERT_EQUAL_FLOAT(500.0f, (float)a); }

void test_state_defaults_safe(void) {
    VehicleState s;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)s.throttle_pct);
    TEST_ASSERT_FALSE(s.throttle_signal_valid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, (float)s.torque_L);
    TEST_ASSERT_FALSE(s.brake_active);
    TEST_ASSERT_FALSE(s.controller_feedback_fresh);
    TEST_ASSERT_TRUE(s.gear == Gear::Neutral);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_clamps_above_max);
    RUN_TEST(test_clamps_below_min);
    RUN_TEST(test_passes_in_range);
    RUN_TEST(test_amp_clamps_above_max);
    RUN_TEST(test_amp_clamps_below_min);
    RUN_TEST(test_amp_allows_continuous_limit);
    RUN_TEST(test_amp_allows_peak_limit);
    RUN_TEST(test_assignment_clamps);
    RUN_TEST(test_default_is_zero);
    RUN_TEST(test_state_defaults_safe);
    RUN_TEST(test_float_does_not_implicitly_become_a_domain_type);
    RUN_TEST(test_domain_type_does_not_implicitly_become_float);
    RUN_TEST(test_domain_types_do_not_launder_through_float);
    RUN_TEST(test_explicit_construction_and_cast_still_work);
    RUN_TEST(test_raw_float_cannot_be_assigned_into_a_domain_type);
    RUN_TEST(test_value_initialisation_survives);
    return UNITY_END();
}
