#include "modules/direction_interlock.h"

DirectionInterlockOutput direction_interlock_update(
    Gear selected, bool throttle_released, bool stopped,
    unsigned required_release_samples, DirectionInterlockState &state) {
    const bool propulsion_gear =
        selected == Gear::Drive || selected == Gear::Reverse;

    if (!propulsion_gear) {
        state.observed = selected;
        state.armed = Gear::Neutral;
        state.release_samples = 0;
        return {false, 0.0f};
    }

    if (selected != state.observed) {
        state.observed = selected;
        state.armed = Gear::Neutral;
        state.release_samples = 0;
    }

    if (state.armed != selected) {
        if (throttle_released && stopped) {
            if (state.release_samples < required_release_samples) {
                ++state.release_samples;
            }
            if (required_release_samples == 0 ||
                state.release_samples >= required_release_samples) {
                state.armed = selected;
            }
        } else {
            state.release_samples = 0;
        }
    }

    if (state.armed != selected) return {false, 0.0f};
    return {true, selected == Gear::Reverse ? -1.0f : 1.0f};
}

