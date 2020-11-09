#include "aos/events/logging/logfile_utils.h"

#include <chrono>
#include <string>

#include "aos/events/logging/logfile_sorting.h"
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

// Tests that we explode when messages are too far out of order.
TEST(PartsMessageReaderDeathTest, TooFarOutOfOrder) {
  const std::string logfile0 = aos::testing::TestTmpDir() + "/log0.bfbs";
  unlink(logfile0.c_str());

  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> config0 =
      JsonToSizedFlatbuffer<LogFileHeader>(
          R"({
  "max_out_of_order_duration": 100000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "parts_uuid": "2a05d725-5d5c-4c0b-af42-88de2f3c3876",
  "parts_index": 0
})");

  const aos::SizePrefixedFlatbufferDetachedBuffer<MessageHeader> m1 =
      JsonToSizedFlatbuffer<MessageHeader>(
          R"({ "channel_index": 0, "monotonic_sent_time": 100000000 })");
  const aos::SizePrefixedFlatbufferDetachedBuffer<MessageHeader> m2 =
      JsonToSizedFlatbuffer<MessageHeader>(
          R"({ "channel_index": 0, "monotonic_sent_time": 0 })");
  const aos::SizePrefixedFlatbufferDetachedBuffer<MessageHeader> m3 =
      JsonToSizedFlatbuffer<MessageHeader>(
          R"({ "channel_index": 0, "monotonic_sent_time": -1 })");

  {
    DetachedBufferWriter writer(logfile0, std::make_unique<DummyEncoder>());
    writer.QueueSpan(config0.full_span());
    writer.QueueSpan(m1.full_span());
    writer.QueueSpan(m2.full_span());
    writer.QueueSpan(m3.full_span());
  }

  const std::vector<LogFile> parts = SortParts({logfile0});

  PartsMessageReader reader(parts[0].parts[0]);

  EXPECT_TRUE(reader.ReadMessage());
  EXPECT_TRUE(reader.ReadMessage());
  EXPECT_DEATH({ reader.ReadMessage(); }, "-0.000000001sec vs. 0.000000000sec");
}

// Tests that we can transparently re-assemble part files with a
// PartsMessageReader.
TEST(PartsMessageReaderTest, ReadWrite) {
  const std::string logfile0 = aos::testing::TestTmpDir() + "/log0.bfbs";
  const std::string logfile1 = aos::testing::TestTmpDir() + "/log1.bfbs";
  unlink(logfile0.c_str());
  unlink(logfile1.c_str());

  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> config0 =
      JsonToSizedFlatbuffer<LogFileHeader>(
          R"({
  "max_out_of_order_duration": 100000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "parts_uuid": "2a05d725-5d5c-4c0b-af42-88de2f3c3876",
  "parts_index": 0
})");
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> config1 =
      JsonToSizedFlatbuffer<LogFileHeader>(
          R"({
  "max_out_of_order_duration": 200000000,
  "monotonic_start_time": 0,
  "realtime_start_time": 0,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "parts_uuid": "2a05d725-5d5c-4c0b-af42-88de2f3c3876",
  "parts_index": 1
})");

  const aos::SizePrefixedFlatbufferDetachedBuffer<MessageHeader> m1 =
      JsonToSizedFlatbuffer<MessageHeader>(
          R"({ "channel_index": 0, "monotonic_sent_time": 1 })");
  const aos::SizePrefixedFlatbufferDetachedBuffer<MessageHeader> m2 =
      JsonToSizedFlatbuffer<MessageHeader>(
          R"({ "channel_index": 0, "monotonic_sent_time": 2 })");

  {
    DetachedBufferWriter writer(logfile0, std::make_unique<DummyEncoder>());
    writer.QueueSpan(config0.full_span());
    writer.QueueSpan(m1.full_span());
  }
  {
    DetachedBufferWriter writer(logfile1, std::make_unique<DummyEncoder>());
    writer.QueueSpan(config1.full_span());
    writer.QueueSpan(m2.full_span());
  }

  const std::vector<LogFile> parts = SortParts({logfile0, logfile1});

  PartsMessageReader reader(parts[0].parts[0]);

  EXPECT_EQ(reader.filename(), logfile0);

  // Confirm that the timestamps track, and the filename also updates.
  // Read the first message.
  EXPECT_EQ(reader.newest_timestamp(), monotonic_clock::min_time);
  EXPECT_EQ(
      reader.max_out_of_order_duration(),
      std::chrono::nanoseconds(config0.message().max_out_of_order_duration()));
  EXPECT_TRUE(reader.ReadMessage());
  EXPECT_EQ(reader.filename(), logfile0);
  EXPECT_EQ(reader.newest_timestamp(),
            monotonic_clock::time_point(chrono::nanoseconds(1)));
  EXPECT_EQ(
      reader.max_out_of_order_duration(),
      std::chrono::nanoseconds(config0.message().max_out_of_order_duration()));

  // Read the second message.
  EXPECT_TRUE(reader.ReadMessage());
  EXPECT_EQ(reader.filename(), logfile1);
  EXPECT_EQ(reader.newest_timestamp(),
            monotonic_clock::time_point(chrono::nanoseconds(2)));
  EXPECT_EQ(
      reader.max_out_of_order_duration(),
      std::chrono::nanoseconds(config1.message().max_out_of_order_duration()));

  // And then confirm that reading again returns no message.
  EXPECT_FALSE(reader.ReadMessage());
  EXPECT_EQ(reader.filename(), logfile1);
  EXPECT_EQ(
      reader.max_out_of_order_duration(),
      std::chrono::nanoseconds(config1.message().max_out_of_order_duration()));
  EXPECT_EQ(reader.newest_timestamp(), monotonic_clock::max_time);
}

}  // namespace testing
}  // namespace logger
}  // namespace aos
