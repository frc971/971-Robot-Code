#include "aos/events/logging/buffer_encoder_param_test.h"

#include "gmock/gmock.h"

namespace aos::logger::testing {

// Verifies that Clear affects the sizes as expected.
TEST_P(BufferEncoderTest, ClearAndSizes) {
  const auto encoder = MakeEncoder();

  std::uniform_int_distribution<int> quantity_distribution(500, 600);
  const int first_quantity = quantity_distribution(*random_number_generator());
  const int second_quantity = quantity_distribution(*random_number_generator());

  const auto first_buffers = CreateAndEncode(first_quantity, encoder.get());
  const auto first_size = encoder->queued_bytes();
  // We want at least 2 buffers to ensure this test is actually validating the
  // behavior.
  ASSERT_GT(first_size, 2u) << ": Encoder chunk size is too big for this test";
  ASSERT_EQ(encoder->total_bytes(), first_size);
  ASSERT_EQ(TotalSize(encoder->queue()), first_size);
  ASSERT_EQ(encoder->queue_size(), encoder->queue().size());

  encoder->Clear(encoder->queue_size());
  ASSERT_EQ(encoder->queued_bytes(), 0);
  ASSERT_EQ(encoder->total_bytes(), first_size);
  ASSERT_EQ(encoder->queue().size(), 0);
  ASSERT_EQ(encoder->queue_size(), 0);

  const auto second_buffers = CreateAndEncode(second_quantity, encoder.get());
  const auto second_size = encoder->queued_bytes();
  ASSERT_GT(second_size, 2u) << ": Encoder chunk size is too big for this test";
  ASSERT_EQ(encoder->total_bytes(), first_size + second_size);
  ASSERT_EQ(TotalSize(encoder->queue()), second_size);
  ASSERT_EQ(encoder->queue_size(), encoder->queue().size());
}

// Runs data in randomly-chunked sizes through the encoder and decoder to verify
// it comes back out the same.
TEST_P(BufferEncoderTest, RoundTrip) {
  std::uniform_int_distribution<int> quantity_distribution(20, 60);
  const char *const test_dir = CHECK_NOTNULL(getenv("TEST_TMPDIR"));
  const std::string file_path = std::string(test_dir) + "/foo";

  std::vector<std::vector<uint8_t>> encoded_buffers;
  {
    const int encode_chunks = quantity_distribution(*random_number_generator());
    const auto encoder = MakeEncoder();
    encoded_buffers = CreateAndEncode(encode_chunks, encoder.get());
    encoder->Finish();

    std::ofstream output_file(file_path, std::ios::binary);
    for (auto span : encoder->queue()) {
      output_file.write(reinterpret_cast<const char *>(span.data()),
                        span.size());
    }
  }

  const size_t total_encoded_size = TotalSize(encoded_buffers);

  // Try decoding in multiple random chunkings.
  for (int i = 0; i < 20; ++i) {
    const auto decoder = MakeDecoder(file_path);
    std::vector<std::vector<uint8_t>> decoded_buffers;
    size_t total_decoded_size = 0;
    while (true) {
      const int chunk_size = quantity_distribution(*random_number_generator());
      std::vector<uint8_t> chunk(chunk_size);
      const size_t read_result =
          decoder->Read(chunk.data(), chunk.data() + chunk_size);
      if (read_result + total_decoded_size != total_encoded_size) {
        // We didn't read everything, so we should've read the complete chunk.
        ASSERT_EQ(read_result, chunk_size)
            << "Read " << read_result + total_decoded_size << " of "
            << total_encoded_size << " expected bytes.";
      }
      // Eventually we'll get here, once the decoder is really sure it's done.
      if (read_result == 0) {
        // Sanity check the math in the test code.
        CHECK_EQ(total_decoded_size, TotalSize(decoded_buffers));
        // Bail out because we're done.
        break;
      }
      // If we're at the end, trim off the 0s so our comparison later works out.
      chunk.resize(read_result);
      total_decoded_size += read_result;
      decoded_buffers.emplace_back(std::move(chunk));
    }
    ASSERT_THAT(Flatten(decoded_buffers),
                ::testing::Eq(Flatten(encoded_buffers)));
  }
}

}  // namespace aos::logger::testing
