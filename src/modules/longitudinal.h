#pragma once
// [FILL-IN] Throttle propulsion / throttle-off regen -> signed total phase-current demand (A).

enum class DriveMode { Normal, Efficiency };

struct LongInput {
    float throttle_pct;   // 0..100
    float pack_soc;       // 0..1
    DriveMode mode;
    bool regen_auto_enabled;
    unsigned regen_level = 3; // legacy ON defaults to level 3
    bool brake_active = false;
    bool one_pedal = true;
    bool four_stage = true;
};

float longitudinal_compute(const LongInput &in);   // + = drive, - = regen
// Any available source at/above cutoff blocks regen; no valid source blocks it.
// Stateless: automatically permits regen below cutoff (other gates still apply).
bool regen_voltage_ok(float em_v, bool em_fresh, float pack_v, bool pack_valid);

struct RegenReleaseState { unsigned zero_samples = 0; };
// Called every 10 ms: qualify regen after 100 ms continuously at calibrated 0%.
// Any positive/invalid throttle immediately cancels release qualification.
bool regen_release_update(float throttle_pct, bool valid, RegenReleaseState &state);
