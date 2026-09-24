#pragma once
// [FILL-IN] Throttle propulsion / throttle-off regen -> signed total phase-current demand (A).

enum class DriveMode { Normal, Efficiency };

struct LongInput {
    float throttle_pct;   // 0..100
    float pack_soc;       // 0..1
    DriveMode mode;
    bool regen_auto_enabled;
};

float longitudinal_compute(const LongInput &in);   // + = drive, - = regen

struct RegenReleaseState { unsigned zero_samples = 0; };
// Called every 10 ms: qualify regen after 100 ms continuously at calibrated 0%.
// Any positive/invalid throttle immediately cancels release qualification.
bool regen_release_update(float throttle_pct, bool valid, RegenReleaseState &state);
