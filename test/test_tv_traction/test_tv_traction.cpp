// Stage 4 — 트랙션 한계 테스트   담당: ______
// stub·실구현 모두 성립하는 불변식으로 시작. 구현하며 TODO를 채우세요.
#include <unity.h>
#include "modules/tv/traction.h"

void test_equal_load_equal_limit(void) {
    // 좌우 하중이 같고 횡가속 0이면 좌우 최대토크도 같다.
    WheelLoads fz{ 800.0f, 800.0f };
    MaxTorque m = tv_traction_compute(fz, 0.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, m.max_L, m.max_R);
    TEST_ASSERT_TRUE(m.max_L > 0.0f);
}

// TODO(담당자): 구현 후 아래를 추가하세요.
//  - Fz가 큰 바퀴가 최대토크도 크다 (단조성)
//  - 횡가속 |ay|이 커지면 종방향 최대토크가 줄어든다 (마찰원)
//  - μ(p.mu)에 비례해 최대토크가 커진다
//  - 횡한계 초과 입력에서 sqrt 음수 없이 0으로 막힌다

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_equal_load_equal_limit);
    return UNITY_END();
}
