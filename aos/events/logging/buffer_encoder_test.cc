#include "aos/events/logging/buffer_encoder.h"

#include <algorithm>
#include <fstream>
#include <string>

#include "aos/events/logging/buffer_encoder_param_test.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace aos::logger::testing {

class DummyEncoderTest : public BufferEncoderBaseTest {};

// Tests that buffers are enqueued without any changes.
TEST_F(DummyEncoderTest, QueuesBuffersAsIs) {
  DummyEncoder encoder;
  const auto expected = CreateAndEncode(100, &encoder);

  auto queue = encoder.queue();
  EXPECT_THAT(queue, ::testing::ElementsAreArray(expected));
}

// Checks that DummyDecoder can read into a buffer.
TEST(DummyDecoderTest, ReadsIntoExactBuffer) {
  static const std::string kTestString{"Just some random words."};

  const char *const test_dir = CHECK_NOTNULL(getenv("TEST_TMPDIR"));
  const std::string file_path = std::string(test_dir) + "/foo";
  std::ofstream(file_path, std::ios::binary) << kTestString;

  // Read the contents of the file into the buffer.
  DummyDecoder dummy_decoder(file_path.c_str());
  std::vector<uint8_t> buffer(kTestString.size());
  const size_t count = dummy_decoder.Read(&*buffer.begin(), &*buffer.end());
  ASSERT_EQ(std::string(buffer.data(), buffer.data() + count), kTestString);

  for (int i = 0; i < 10; ++i) {
    // Verify that there is no more data to read from the file.
    ASSERT_EQ(dummy_decoder.Read(&*buffer.begin(), &*buffer.end()), 0);
  }
}

// Checks that DummyDecoder can read into a buffer that can accommodate all the
// data in the file.
TEST(DummyDecoderTest, ReadsIntoLargerBuffer) {
  static const std::string kTestString{"Just some random words."};

  const char *const test_dir = CHECK_NOTNULL(getenv("TEST_TMPDIR"));
  const std::string file_path = std::string(test_dir) + "/foo";
  std::ofstream(file_path, std::ios::binary) << kTestString;

  DummyDecoder dummy_decoder(file_path.c_str());
  std::vector<uint8_t> buffer(100);
  const size_t count = dummy_decoder.Read(&*buffer.begin(), &*buffer.end());
  buffer.resize(count);
  ASSERT_EQ(std::string(buffer.data(), buffer.data() + count), kTestString);

  // Verify that there is no more data to read from the file.
  ASSERT_EQ(dummy_decoder.Read(&*buffer.begin(), &*buffer.end()), 0);
}

// Checks that DummyDecoder can repeatedly read the contents of the file into a
// smaller buffer until there is no more to read.
TEST(DummyDecoderTest, ReadsRepeatedlyIntoSmallerBuffer) {
  static const std::string kTestString{"Just some random words."};

  const char *const test_dir = CHECK_NOTNULL(getenv("TEST_TMPDIR"));
  const std::string file_path = std::string(test_dir) + "/foo";
  std::ofstream(file_path, std::ios::binary) << kTestString;

  DummyDecoder dummy_decoder(file_path.c_str());
  std::vector<uint8_t> buffer((kTestString.size() + 1) / 2);

  {
    // Read into our buffer once, and verify the contents.
    const size_t count = dummy_decoder.Read(&*buffer.begin(), &*buffer.end());
    ASSERT_EQ(std::string(buffer.data(), buffer.data() + count),
              kTestString.substr(0, buffer.size()));
  }

  {
    // Read into the same buffer again, and verify the contents.
    const size_t count = dummy_decoder.Read(&*buffer.begin(), &*buffer.end());
    ASSERT_EQ(
        std::string(buffer.data(), buffer.data() + count),
        kTestString.substr(buffer.size(), kTestString.size() - buffer.size()));
  }

  // Verify that there is no more data to read from the file.
  ASSERT_EQ(dummy_decoder.Read(&*buffer.begin(), &*buffer.end()), 0);
}

INSTANTIATE_TEST_SUITE_P(
    Dummy, BufferEncoderTest,
    ::testing::Combine(::testing::Values([]() {
                         return std::make_unique<DummyEncoder>();
                       }),
                       ::testing::Values([](std::string_view filename) {
                         return std::make_unique<DummyDecoder>(filename);
                       }),
                       ::testing::Range(0, 100)));

}  // namespace aos::logger::testing
