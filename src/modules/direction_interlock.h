#pragma once

#include "modules/gear.h"

struct DirectionInterlockState {
    Gear observed = Gear::Neutral;
    Gear armed = Gear::Neutral;
    unsigned release_samples = 0;
};

struct DirectionInterlockOutput {
    bool propulsion_enabled;
    float command_sign;
};

// A newly selected Drive or Reverse direction is armed only after the vehicle
// is stopped and the throttle has remained released for the requested number
// of scheduler samples. Neutral/Park/invalid classifications disarm it.
DirectionInterlockOutput direction_interlock_update(
    Gear selected, bool throttle_released, bool stopped,
    unsigned required_release_samples, DirectionInterlockState &state);

