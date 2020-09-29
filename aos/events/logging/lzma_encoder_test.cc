#include "aos/events/logging/lzma_encoder.h"

#include "aos/events/logging/buffer_encoder_param_test.h"
#include "gtest/gtest.h"

namespace aos::logger::testing {

INSTANTIATE_TEST_CASE_P(
    Lzma, BufferEncoderTest,
    ::testing::Combine(::testing::Values([]() {
                         return std::make_unique<LzmaEncoder>(2);
                       }),
                       ::testing::Values([](std::string_view filename) {
                         return std::make_unique<LzmaDecoder>(filename);
                       }),
                       ::testing::Range(0, 100)));

}  // namespace aos::logger::testing
