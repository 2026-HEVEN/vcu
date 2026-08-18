#include <unity.h>
#include "modules/gear.h"

// 예시 저항 사다리(placeholder): R≈500, N≈2000, D≈3500, 밴드 ±300 (실측 전 임시)
static GearCalib cal() { return { 500, 2000, 3500, 300 }; }

void test_reverse_band(void) { TEST_ASSERT_TRUE(gear_compute({ 520 },  cal()) == Gear::Reverse); }
void test_neutral_band(void) { TEST_ASSERT_TRUE(gear_compute({ 1950 }, cal()) == Gear::Neutral); }
void test_drive_band(void)   { TEST_ASSERT_TRUE(gear_compute({ 3450 }, cal()) == Gear::Drive);   }

// 밴드 밖(중간위치·플로팅·단선) → 안전상 Neutral
void test_gap_is_neutral(void) {
    TEST_ASSERT_TRUE(gear_compute({ 1200 }, cal()) == Gear::Neutral);   // R·N 사이 갭
    TEST_ASSERT_TRUE(gear_compute({ 4095 }, cal()) == Gear::Neutral);   // 레일(단선)
    TEST_ASSERT_TRUE(gear_compute({ 0 },    cal()) == Gear::Neutral);   // 레일(GND)
}

// 밴드 경계에서 가장 가까운 위치가 선택된다
void test_nearest_wins(void) {
    TEST_ASSERT_TRUE(gear_compute({ 2250 }, cal()) == Gear::Neutral);   // 2000에 250 (tol내)
    TEST_ASSERT_TRUE(gear_compute({ 3250 }, cal()) == Gear::Drive);     // 3500에 250
    TEST_ASSERT_TRUE(gear_compute({ 750 },  cal()) == Gear::Reverse);   // 500에 250
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_reverse_band);
    RUN_TEST(test_neutral_band);
    RUN_TEST(test_drive_band);
    RUN_TEST(test_gap_is_neutral);
    RUN_TEST(test_nearest_wins);
    return UNITY_END();
}
