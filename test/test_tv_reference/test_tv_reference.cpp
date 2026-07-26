// Stage 1 — 레퍼런스 모델 테스트   담당: ______
// 아래는 stub·실구현 모두에서 성립하는 불변식. 구현하며 TODO 테스트를 채우세요.
#include <unity.h>
#include "modules/tv/reference.h"

void test_no_steer_no_target(void) {
    // 직진(조향 0)이면 목표 yaw rate는 0.
    float dy = tv_reference_compute(Unit(0.0f), 15.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, dy);
}

// TODO(담당자): 구현 후 아래 같은 테스트를 추가하세요.
//  - 조향이 커지면 |목표 yaw|도 커진다 (단조성)
//  - 같은 조향에서 차속↑이면 목표 yaw 변화 (모델 특성 확인)
//  - 목표 yaw가 p.desired_yaw_max를 넘지 않는다 (clamp)
//  - 좌/우 조향의 부호가 IMU yaw 규약과 일치한다

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_steer_no_target);
    return UNITY_END();
}
