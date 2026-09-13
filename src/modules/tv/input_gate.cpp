#include "modules/tv/input_gate.h"

TVInputGateOutput tv_input_gate_compute(const TVInputGateInput &in,
                                        TVInputGateState &s) {
    TVInputGateOutput out{};
    // imu_frame_fresh alone accepts any MTData2 frame, even one without the
    // rate-of-turn group, which would freeze yaw_rate while TV stays enabled.
    // An untrusted steering input would make the reference yaw arbitrary.
    out.tv_enable = in.tv_requested && in.imu_frame_fresh &&
                    in.yaw_sample_fresh && in.steering_valid;

    // MTi frames are not locked to the 10 ms control task. An identical value
    // is a re-read of the same sample, so the yaw D term can use the real
    // sample interval instead of seeing D=0 and then a doubled step.
    out.yaw_sample_repeated =
        s.have_last_yaw && in.yaw_rate_dps == s.last_yaw_rate_dps;
    s.last_yaw_rate_dps = in.yaw_rate_dps;
    s.have_last_yaw = true;
    return out;
}
