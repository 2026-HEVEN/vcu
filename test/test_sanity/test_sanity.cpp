#include <unity.h>

void test_harness_runs(void) { TEST_ASSERT_EQUAL_INT(2, 1 + 1); }

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_harness_runs);
    return UNITY_END();
}
