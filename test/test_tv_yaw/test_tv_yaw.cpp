// Stage 2 — yaw 제어기 테스트   담당: ______
// stub·실구현 모두 성립하는 불변식으로 시작. 구현하며 TODO를 채우세요.
#include <unity.h>
#include "modules/tv/yaw_control.h"

void test_zero_error_zero_moment(void) {
    // 목표 == 실측 (오차 0), 갓 초기화한 상태면 Mz는 0.
    TVYawState s{};
    float mz = tv_yaw_compute(10.0f, 10.0f, 0.01f, TV_PARAMS, s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, mz);
}

// TODO(담당자): 구현 후 아래를 추가하세요.
//  - 양(+) 오차가 지속되면 적분항이 쌓여 Mz가 커진다
//  - Mz는 p.yaw_moment_max로 clamp 된다 (와인드업/출력 포화)
//  - 상태 s를 주입해 적분 누적이 결정론적으로 재현되는지
//  - dt=0 방어 (0으로 나눔 없음)

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_error_zero_moment);
    return UNITY_END();
}
