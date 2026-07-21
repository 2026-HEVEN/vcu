// Stage 5 — 토크 배분 테스트   담당: ______
// stub·실구현 모두 성립하는 불변식으로 시작. 구현하며 TODO를 채우세요.
#include <unity.h>
#include "modules/tv/allocation.h"

static MaxTorque unlimited() { return { 1.0e6f, 1.0e6f }; }

void test_no_yaw_is_symmetric(void) {
    // Mz=0, 상한 넉넉하면 좌우 균등 & 합=총토크.
    TVAllocOutput o = tv_alloc_compute(20.0f, 0.0f, unlimited());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, (float)o.torque_L, (float)o.torque_R);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, (float)o.torque_L + (float)o.torque_R);
}

// TODO(담당자): 구현 후 아래를 추가하세요.
//  - Mz>0 이면 좌우 토크에 차등이 생긴다 (부호 규약 확인)
//  - 트랙션 상한(limit)을 넘는 배분은 상한으로 clamp 된다
//  - 한쪽이 상한에 걸릴 때의 정책(총량 유지 vs yaw 유지)이 의도대로 동작
//  - 회생(총토크 음수) 구간에서도 부호가 올바르다

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_yaw_is_symmetric);
    return UNITY_END();
}
