#include "aos/events/logging/logfile_utils.h"

#include <chrono>
#include <string>

#include "aos/events/logging/test_message_generated.h"
#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/tmpdir.h"
#include "gtest/gtest.h"

namespace aos {
namespace logger {
namespace testing {
namespace chrono = std::chrono;

// Creates a size prefixed flatbuffer from json.
template <typename T>
SizePrefixedFlatbufferDetachedBuffer<T> JsonToSizedFlatbuffer(
    const std::string_view data) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  fbb.FinishSizePrefixed(JsonToFlatbuffer<T>(data, &fbb));
  return fbb.Release();
}

// Tests that we can write and read 2 flatbuffers to file.
TEST(SpanReaderTest, ReadWrite) {
  const std::string logfile = aos::testing::TestTmpDir() + "/log.bfbs";
  unlink(logfile.c_str());

  const aos::SizePrefixedFlatbufferDetachedBuffer<TestMessage> m1 =
      JsonToSizedFlatbuffer<TestMessage>(
          R"({ "value": 1 })");
  const aos::SizePrefixedFlatbufferDetachedBuffer<TestMessage> m2 =
      JsonToSizedFlatbuffer<TestMessage>(
          R"({ "value": 2 })");

  {
    DetachedBufferWriter writer(logfile, std::make_unique<DummyEncoder>());
    writer.QueueSpan(m1.full_span());
    writer.QueueSpan(m2.full_span());
  }

  SpanReader reader(logfile);

  EXPECT_EQ(reader.filename(), logfile);
  EXPECT_EQ(reader.ReadMessage(), m1.full_span());
  EXPECT_EQ(reader.ReadMessage(), m2.full_span());
  EXPECT_EQ(reader.ReadMessage(), absl::Span<const uint8_t>());
}

// Tests that we can actually parse the resulting messages at a basic level
// through MessageReader.
TEST(MessageReaderTest, ReadWrite) {
  const std::string logfile = aos::testing::TestTmpDir() + "/log.bfbs";
  unlink(logfile.c_str());

  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> config =
      JsonToSizedFlatbuffer<LogFileHeader>(
          R"({ "max_out_of_order_duration": 100000000 })");
  const aos::SizePrefixedFlatbufferDetachedBuffer<MessageHeader> m1 =
      JsonToSizedFlatbuffer<MessageHeader>(
          R"({ "channel_index": 0, "monotonic_sent_time": 1 })");
  const aos::SizePrefixedFlatbufferDetachedBuffer<MessageHeader> m2 =
      JsonToSizedFlatbuffer<MessageHeader>(
          R"({ "channel_index": 0, "monotonic_sent_time": 2 })");

  {
    DetachedBufferWriter writer(logfile, std::make_unique<DummyEncoder>());
    writer.QueueSpan(config.full_span());
    writer.QueueSpan(m1.full_span());
    writer.QueueSpan(m2.full_span());
  }

  MessageReader reader(logfile);

  EXPECT_EQ(reader.filename(), logfile);

  EXPECT_EQ(
      reader.max_out_of_order_duration(),
      std::chrono::nanoseconds(config.message().max_out_of_order_duration()));
  EXPECT_EQ(reader.newest_timestamp(), monotonic_clock::min_time);
  EXPECT_TRUE(reader.ReadMessage());
  EXPECT_EQ(reader.newest_timestamp(),
            monotonic_clock::time_point(chrono::nanoseconds(1)));
  EXPECT_TRUE(reader.ReadMessage());
  EXPECT_EQ(reader.newest_timestamp(),
            monotonic_clock::time_point(chrono::nanoseconds(2)));
  EXPECT_FALSE(reader.ReadMessage());
}

}  // namespace testing
}  // namespace logger
}  // namespace aos
