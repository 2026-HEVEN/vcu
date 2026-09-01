#pragma once
// [FILL-IN] Longitudinal (accel/brake/regen) strategy -> signed total torque demand (A).

enum class DriveMode { Normal, Efficiency };

struct LongInput {
    float throttle_pct;   // 0..100
    float brake_pct;      // 0..100
    float pack_soc;       // 0..1
    DriveMode mode;
    bool regen_auto_enabled;
};

float longitudinal_compute(const LongInput &in);   // + = drive, - = regen
