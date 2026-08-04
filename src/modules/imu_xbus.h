#pragma once

#include <cstddef>
#include <cstdint>
#include "modules/imu.h"

// Arduino-independent Xbus/MTData2 parser. A sample becomes valid only when a
// checksum-valid frame contains both Acceleration and RateOfTurn as Float32.
class ImuXbusParser {
public:
    void reset();
    bool feed(uint8_t byte);

    bool has_sample() const { return has_sample_; }
    const ImuRaw &sample() const { return latest_; }

private:
    enum class State {
        WaitPreamble,
        WaitBusId,
        WaitMessageId,
        WaitLength,
        WaitPayload,
        WaitChecksum,
    };

    bool parse_mtdata2();

    State state_ = State::WaitPreamble;
    uint8_t message_id_ = 0;
    uint8_t length_ = 0;
    uint16_t index_ = 0;
    uint8_t checksum_ = 0;
    uint8_t payload_[254]{};
    ImuRaw latest_{0.0f, 0.0f, 0.0f};
    bool has_sample_ = false;
};
