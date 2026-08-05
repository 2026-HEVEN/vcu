#pragma once
// [FILL-IN] Longitudinal (accel/brake/regen) strategy -> signed total torque demand (A).

enum class DriveMode { Normal, Efficiency };

struct LongInput {
    float throttle_pct;   // 0..100
    float brake_pct;      // 0..100
    float pack_soc;       // 0..1
    float motor_rpm;      // absolute motor speed, RPM
    DriveMode mode;
};

struct LongitudinalState {
    float regen_current_a = 0.0f;  // slew-limited regen magnitude
};

float longitudinal_compute(const LongInput &in,
                           LongitudinalState &state,
                           float dt_s);   // + = drive, - = regen
