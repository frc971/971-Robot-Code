#include "aos/events/logging/snappy_encoder.h"

#include "aos/events/logging/buffer_encoder_param_test.h"
#include "aos/util/file.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace aos::logger::testing {

INSTANTIATE_TEST_SUITE_P(
    Snappy, BufferEncoderTest,
    ::testing::Combine(::testing::Values([](size_t max_message_size) {
                         return std::make_unique<SnappyEncoder>(max_message_size);
                       }),
                       ::testing::Values([](std::string_view filename) {
                         return std::make_unique<SnappyDecoder>(filename);
                       }),
                       ::testing::Range(0, 100)));

}  // namespace aos::logger::testing
