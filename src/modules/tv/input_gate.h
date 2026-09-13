#pragma once
// TV 입력 게이트. 어떤 센서 상태에서 TV를 켜도 되는지 판단한다.
// 코어 배선(app_wiring)은 결과를 TVInput에 그대로 넘기기만 하고, 판단 로직은
// 모듈에 두어 native 테스트로 지킨다.

struct TVInputGateInput {
    bool  tv_requested;      // 대시 TC/TV 스위치 요청 (Cluster 명령)
    bool  imu_frame_fresh;   // MTData2 프레임이 최근에 수신됨 (imu_valid)
    bool  yaw_sample_fresh;  // rate-of-turn 그룹이 실제로 최근에 수신됨
    bool  steering_valid;    // steering_sample_valid() 결과
    float yaw_rate_dps;      // 이번 tick yaw rate [deg/s]
    // 주의: 기본 멤버 초기화자 금지 (app_wiring이 리스트 초기화로 채운다).
};

struct TVInputGateOutput {
    bool tv_enable;            // -> TVInput.tv_enable_requested
    bool yaw_sample_repeated;  // -> TVInput.yaw_sample_repeated
};

struct TVInputGateState {
    float last_yaw_rate_dps = 0.0f;
    bool  have_last_yaw     = false;
};

TVInputGateOutput tv_input_gate_compute(const TVInputGateInput &in,
                                        TVInputGateState &s);
