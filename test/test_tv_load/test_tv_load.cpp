// Stage 3 — 하중 추정 테스트   담당: ______
// stub·실구현 모두 성립하는 불변식으로 시작. 구현하며 TODO를 채우세요.
#include <unity.h>
#include "modules/tv/load.h"

void test_no_accel_symmetric_load(void) {
    // 가속 없음(ax=ay=0)이면 좌우 하중은 대칭.
    WheelLoads fz = tv_load_compute(0.0f, 0.0f, TV_PARAMS);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, fz.fz_L, fz.fz_R);
    TEST_ASSERT_TRUE(fz.fz_L > 0.0f);   // 정적 하중은 양수
}

// TODO(담당자): 구현 후 아래를 추가하세요.
//  - 횡가속 ay>0 이면 바깥 바퀴 Fz가 안쪽보다 크다 (하중 이동 방향)
//  - 좌우 Fz 합이 대략 (구동축 정적하중) 근처로 보존된다
//  - 큰 ay에서 안쪽 Fz가 0으로 clamp (바퀴 들림) 되고 음수가 안 나온다
//  - ax(종가속)에 따른 구동축 하중 변화가 부호대로 반영된다

void setUp(void) {}
void tearDown(void) {}
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_accel_symmetric_load);
    return UNITY_END();
}
