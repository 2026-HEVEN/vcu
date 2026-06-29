#include <unity.h>
#include "modules/imu.h"

void test_first_sample_seeds(void) {
    ImuFilterState s{};
    ImuOutput o = imu_compute({10.0f, 0.0f, 0.0f}, s);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, o.yaw_rate);   // seeds to first reading
    TEST_ASSERT_TRUE(s.seeded);
}
void test_lowpass_smooths(void) {
    ImuFilterState s{};
    imu_compute({0.0f, 0, 0}, s);                 // seed at 0
    ImuOutput o = imu_compute({100.0f, 0, 0}, s); // step input is attenuated
    TEST_ASSERT_TRUE(o.yaw_rate > 0.0f && o.yaw_rate < 100.0f);
}

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_first_sample_seeds);
    RUN_TEST(test_lowpass_smooths);
    return UNITY_END();
}
