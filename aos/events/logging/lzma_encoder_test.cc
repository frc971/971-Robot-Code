#include "aos/events/logging/lzma_encoder.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/events/logging/buffer_encoder_param_test.h"
#include "aos/util/file.h"

DECLARE_int32(lzma_threads);

namespace aos::logger::testing {

INSTANTIATE_TEST_SUITE_P(
    MtLzma, BufferEncoderTest,
    ::testing::Combine(::testing::Values([](size_t max_message_size) {
                         FLAGS_lzma_threads = 3;
                         return std::make_unique<LzmaEncoder>(max_message_size,
                                                              2, 4096);
                       }),
                       ::testing::Values([](std::string_view filename) {
                         return std::make_unique<LzmaDecoder>(filename);
                       }),
                       ::testing::Range(0, 100)));

INSTANTIATE_TEST_SUITE_P(
    MtLzmaThreaded, BufferEncoderTest,
    ::testing::Combine(::testing::Values([](size_t max_message_size) {
                         FLAGS_lzma_threads = 3;
                         return std::make_unique<LzmaEncoder>(max_message_size,
                                                              5, 4096);
                       }),
                       ::testing::Values([](std::string_view filename) {
                         return std::make_unique<ThreadedLzmaDecoder>(filename);
                       }),
                       ::testing::Range(0, 100)));

INSTANTIATE_TEST_SUITE_P(
    Lzma, BufferEncoderTest,
    ::testing::Combine(::testing::Values([](size_t max_message_size) {
                         FLAGS_lzma_threads = 1;
                         return std::make_unique<LzmaEncoder>(max_message_size,
                                                              2, 4096);
                       }),
                       ::testing::Values([](std::string_view filename) {
                         return std::make_unique<LzmaDecoder>(filename);
                       }),
                       ::testing::Range(0, 100)));

INSTANTIATE_TEST_SUITE_P(
    LzmaThreaded, BufferEncoderTest,
    ::testing::Combine(::testing::Values([](size_t max_message_size) {
                         FLAGS_lzma_threads = 1;
                         return std::make_unique<LzmaEncoder>(max_message_size,
                                                              5, 4096);
                       }),
                       ::testing::Values([](std::string_view filename) {
                         return std::make_unique<ThreadedLzmaDecoder>(filename);
                       }),
                       ::testing::Range(0, 100)));

// Tests that we return as much of the file as we can read if the end is
// corrupted.
TEST_F(BufferEncoderBaseTest, CorruptedBuffer) {
  std::uniform_int_distribution<int> quantity_distribution(20, 60);
  const char *const test_dir = CHECK_NOTNULL(getenv("TEST_TMPDIR"));
  const std::string file_path = std::string(test_dir) + "/foo";

  std::vector<std::vector<uint8_t>> encoded_buffers;
  {
    const int encode_chunks = quantity_distribution(*random_number_generator());
    const auto encoder = std::make_unique<LzmaEncoder>(
        BufferEncoderBaseTest::kMaxMessageSize, 2);
    encoded_buffers = CreateAndEncode(encode_chunks, encoder.get());
    encoder->Finish();

    std::string contents = "";
    for (auto span : encoder->queue()) {
      absl::StrAppend(
          &contents,
          std::string_view(reinterpret_cast<const char *>(span.data()),
                           span.size()));
    }
    aos::util::WriteStringToFileOrDie(
        file_path, contents.substr(0, contents.size() - 200));
  }

  const size_t total_encoded_size = TotalSize(encoded_buffers);

  // Try decoding in multiple random chunkings.
  for (int i = 0; i < 20; ++i) {
    const auto decoder = std::make_unique<LzmaDecoder>(file_path);
    std::vector<std::vector<uint8_t>> decoded_buffers;
    size_t total_decoded_size = 0;
    while (true) {
      const int chunk_size = quantity_distribution(*random_number_generator());
      std::vector<uint8_t> chunk(chunk_size);
      const size_t read_result =
          decoder->Read(chunk.data(), chunk.data() + chunk_size);
      // Eventually we'll get here, once the decoder is really sure it's done.
      if (read_result == 0) {
        // Sanity check the math in the test code.
        LOG(INFO) << "Decoded " << total_decoded_size << " encoded "
                  << total_encoded_size;
        CHECK_EQ(total_decoded_size, TotalSize(decoded_buffers));
        break;
      }
      // If we're at the end, trim off the 0s so our comparison later works out.
      chunk.resize(read_result);
      total_decoded_size += read_result;
      decoded_buffers.emplace_back(std::move(chunk));
    }
    auto flattened_encoded = Flatten(encoded_buffers);
    auto flattened_decoded = Flatten(decoded_buffers);

    ASSERT_LE(flattened_decoded.size(), flattened_encoded.size());
    flattened_encoded.resize(flattened_decoded.size());

    ASSERT_THAT(flattened_decoded, ::testing::Eq(flattened_encoded));
  }
}

}  // namespace aos::logger::testing
