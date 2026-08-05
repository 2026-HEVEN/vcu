#include <unity.h>

#include <cstdint>
#include <cstring>
#include <vector>
#include "modules/imu_xbus.h"

namespace {
void append_be_float(std::vector<uint8_t> &out, float value) {
    uint8_t bytes[4];
    std::memcpy(bytes, &value, sizeof(value));
    out.push_back(bytes[3]);
    out.push_back(bytes[2]);
    out.push_back(bytes[1]);
    out.push_back(bytes[0]);
}

void append_vector(std::vector<uint8_t> &payload,
                   uint16_t data_id,
                   float x,
                   float y,
                   float z) {
    payload.push_back(static_cast<uint8_t>(data_id >> 8U));
    payload.push_back(static_cast<uint8_t>(data_id));
    payload.push_back(12U);
    append_be_float(payload, x);
    append_be_float(payload, y);
    append_be_float(payload, z);
}

std::vector<uint8_t> frame(const std::vector<uint8_t> &payload,
                           uint8_t message_id = 0x36U) {
    std::vector<uint8_t> result{0xFAU, 0xFFU, message_id,
                                static_cast<uint8_t>(payload.size())};
    result.insert(result.end(), payload.begin(), payload.end());
    uint8_t sum = 0;
    for (std::size_t i = 1; i < result.size(); ++i) sum += result[i];
    result.push_back(static_cast<uint8_t>(0U - sum));
    return result;
}

bool feed(ImuXbusParser &parser, const std::vector<uint8_t> &bytes) {
    bool completed = false;
    for (uint8_t byte : bytes) completed = parser.feed(byte) || completed;
    return completed;
}

std::vector<uint8_t> complete_payload() {
    std::vector<uint8_t> payload;
    append_vector(payload, 0x4020U, 9.80665f, -4.903325f, 0.0f);
    append_vector(payload, 0x8020U, 0.0f, 0.0f, 1.0f);
    return payload;
}
} // namespace

void test_parses_complete_mtdata2_sample(void) {
    ImuXbusParser parser;
    TEST_ASSERT_TRUE(feed(parser, frame(complete_payload())));
    TEST_ASSERT_TRUE(parser.has_sample());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, parser.sample().accel_x);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f, parser.sample().accel_y);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 57.29578f, parser.sample().yaw_rate);
}

void test_rejects_bad_checksum(void) {
    ImuXbusParser parser;
    std::vector<uint8_t> bytes = frame(complete_payload());
    bytes.back()++;
    TEST_ASSERT_FALSE(feed(parser, bytes));
    TEST_ASSERT_FALSE(parser.has_sample());
}

void test_rejects_frame_missing_rate_of_turn(void) {
    ImuXbusParser parser;
    std::vector<uint8_t> payload;
    append_vector(payload, 0x4020U, 1.0f, 2.0f, 3.0f);
    TEST_ASSERT_FALSE(feed(parser, frame(payload)));
    TEST_ASSERT_FALSE(parser.has_sample());
}

void test_rejects_malformed_tlv_length(void) {
    ImuXbusParser parser;
    const std::vector<uint8_t> payload{0x40U, 0x20U, 12U, 0U};
    TEST_ASSERT_FALSE(feed(parser, frame(payload)));
    TEST_ASSERT_FALSE(parser.has_sample());
}

void test_resynchronizes_after_noise_and_wrong_mid(void) {
    ImuXbusParser parser;
    TEST_ASSERT_FALSE(feed(parser, {0x01U, 0x02U, 0x03U}));
    TEST_ASSERT_FALSE(feed(parser, frame(complete_payload(), 0x35U)));
    TEST_ASSERT_TRUE(feed(parser, frame(complete_payload())));
}

void test_reset_invalidates_previous_sample(void) {
    ImuXbusParser parser;
    TEST_ASSERT_TRUE(feed(parser, frame(complete_payload())));
    parser.reset();
    TEST_ASSERT_FALSE(parser.has_sample());
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_parses_complete_mtdata2_sample);
    RUN_TEST(test_rejects_bad_checksum);
    RUN_TEST(test_rejects_frame_missing_rate_of_turn);
    RUN_TEST(test_rejects_malformed_tlv_length);
    RUN_TEST(test_resynchronizes_after_noise_and_wrong_mid);
    RUN_TEST(test_reset_invalidates_previous_sample);
    return UNITY_END();
}
