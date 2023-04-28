#include "aos/events/logging/logfile_utils.h"

#include <chrono>
#include <filesystem>
#include <random>
#include <string>

#include "absl/strings/escaping.h"
#include "external/com_github_google_flatbuffers/src/annotated_binary_text_gen.h"
#include "external/com_github_google_flatbuffers/src/binary_annotator.h"
#include "flatbuffers/reflection_generated.h"
#include "gflags/gflags.h"
#include "gtest/gtest.h"

#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/test_message_generated.h"
#include "aos/flatbuffer_merge.h"
#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/path.h"
#include "aos/testing/random_seed.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"

namespace aos {
namespace logger {
namespace testing {
namespace chrono = std::chrono;
using aos::message_bridge::RemoteMessage;
using aos::testing::ArtifactPath;

// Adapter class to make it easy to test DetachedBufferWriter without adding
// test only boilerplate to DetachedBufferWriter.
class TestDetachedBufferWriter : public FileBackend,
                                 public DetachedBufferWriter {
 public:
  // Pick a max size that is rather conservative.
  static constexpr size_t kMaxMessageSize = 128 * 1024;
  TestDetachedBufferWriter(std::string_view filename)
      : FileBackend("/"),
        DetachedBufferWriter(FileBackend::RequestFile(filename),
                             std::make_unique<DummyEncoder>(kMaxMessageSize)) {}
  void WriteSizedFlatbuffer(flatbuffers::DetachedBuffer &&buffer) {
    QueueSpan(absl::Span<const uint8_t>(buffer.data(), buffer.size()));
  }
  void QueueSpan(absl::Span<const uint8_t> buffer) {
    DataEncoder::SpanCopier coppier(buffer);
    CopyMessage(&coppier, monotonic_clock::now());
  }
};

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
      JsonToSizedFlatbuffer<TestMessage>(R"({ "value": 1 })");
  const aos::SizePrefixedFlatbufferDetachedBuffer<TestMessage> m2 =
      JsonToSizedFlatbuffer<TestMessage>(R"({ "value": 2 })");

  {
    TestDetachedBufferWriter writer(logfile);
    writer.QueueSpan(m1.span());
    writer.QueueSpan(m2.span());
  }

  SpanReader reader(logfile);

  EXPECT_EQ(reader.filename(), logfile);
  EXPECT_EQ(reader.PeekMessage(), m1.span());
  EXPECT_EQ(reader.PeekMessage(), m1.span());
  EXPECT_EQ(reader.ReadMessage(), m1.span());
  EXPECT_EQ(reader.ReadMessage(), m2.span());
  EXPECT_EQ(reader.PeekMessage(), absl::Span<const uint8_t>());
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
    TestDetachedBufferWriter writer(logfile);
    writer.QueueSpan(config.span());
    writer.QueueSpan(m1.span());
    writer.QueueSpan(m2.span());
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
  "configuration": {},
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
    TestDetachedBufferWriter writer(logfile0);
    writer.QueueSpan(config0.span());
    writer.QueueSpan(m1.span());
    writer.QueueSpan(m2.span());
    writer.QueueSpan(m3.span());
  }
  ASSERT_TRUE(std::filesystem::exists(logfile0)) << logfile0;

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
  "configuration": {},
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
  "configuration": {},
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
    TestDetachedBufferWriter writer(logfile0);
    writer.QueueSpan(config0.span());
    writer.QueueSpan(m1.span());
  }
  {
    TestDetachedBufferWriter writer(logfile1);
    writer.QueueSpan(config1.span());
    writer.QueueSpan(m2.span());
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

// Tests that Message's operator < works as expected.
TEST(MessageTest, Sorting) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();

  Message m1{.channel_index = 0,
             .queue_index = BootQueueIndex{.boot = 0, .index = 0u},
             .timestamp =
                 BootTimestamp{.boot = 0, .time = e + chrono::milliseconds(1)},
             .monotonic_remote_boot = 0xffffff,
             .monotonic_timestamp_boot = 0xffffff,
             .data = nullptr};
  Message m2{.channel_index = 0,
             .queue_index = BootQueueIndex{.boot = 0, .index = 0u},
             .timestamp =
                 BootTimestamp{.boot = 0, .time = e + chrono::milliseconds(2)},
             .monotonic_remote_boot = 0xffffff,
             .monotonic_timestamp_boot = 0xffffff,
             .data = nullptr};

  EXPECT_LT(m1, m2);
  EXPECT_GE(m2, m1);

  m1.timestamp.time = e;
  m2.timestamp.time = e;

  m1.channel_index = 1;
  m2.channel_index = 2;

  EXPECT_LT(m1, m2);
  EXPECT_GE(m2, m1);

  m1.channel_index = 0;
  m2.channel_index = 0;
  m1.queue_index.index = 0u;
  m2.queue_index.index = 1u;

  EXPECT_LT(m1, m2);
  EXPECT_GE(m2, m1);
}

aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> MakeHeader(
    const aos::FlatbufferDetachedBuffer<Configuration> &config,
    const std::string_view json) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  flatbuffers::Offset<Configuration> config_offset =
      aos::CopyFlatBuffer(config, &fbb);
  LogFileHeader::Builder header_builder(fbb);
  header_builder.add_configuration(config_offset);
  fbb.Finish(header_builder.Finish());
  aos::FlatbufferDetachedBuffer<LogFileHeader> config_header(fbb.Release());

  aos::FlatbufferDetachedBuffer<LogFileHeader> header_updates(
      JsonToFlatbuffer<LogFileHeader>(json));
  CHECK(header_updates.Verify());
  flatbuffers::FlatBufferBuilder fbb2;
  fbb2.ForceDefaults(true);
  fbb2.FinishSizePrefixed(
      aos::MergeFlatBuffers(config_header, header_updates, &fbb2));
  return fbb2.Release();
}

class SortingElementTest : public ::testing::Test {
 public:
  SortingElementTest()
      : config_(JsonToFlatbuffer<Configuration>(
            R"({
  "channels": [
    {
      "name": "/a",
      "type": "aos.logger.testing.TestMessage",
      "source_node": "pi1",
      "destination_nodes": [
        {
          "name": "pi2"
        },
        {
          "name": "pi3"
        }
      ]
    },
    {
      "name": "/b",
      "type": "aos.logger.testing.TestMessage",
      "source_node": "pi1"
    },
    {
      "name": "/c",
      "type": "aos.logger.testing.TestMessage",
      "source_node": "pi1"
    },
    {
      "name": "/d",
      "type": "aos.logger.testing.TestMessage",
      "source_node": "pi2",
      "destination_nodes": [
        {
          "name": "pi1"
        }
      ]
    }
  ],
  "nodes": [
    {
      "name": "pi1"
    },
    {
      "name": "pi2"
    },
    {
      "name": "pi3"
    }
  ]
}
)")),
        config0_(MakeHeader(config_, R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi1"
  },
  "logger_node": {
    "name": "pi1"
  },
  "monotonic_start_time": 1000000,
  "realtime_start_time": 1000000000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "source_node_boot_uuid": "1d782c63-b3c7-466e-bea9-a01308b43333",
  "logger_node_boot_uuid": "1d782c63-b3c7-466e-bea9-a01308b43333",
  "boot_uuids": [
    "1d782c63-b3c7-466e-bea9-a01308b43333",
    "",
    ""
  ],
  "parts_uuid": "2a05d725-5d5c-4c0b-af42-88de2f3c3876",
  "parts_index": 0
})")),
        config1_(MakeHeader(config_,
                            R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi1"
  },
  "logger_node": {
    "name": "pi1"
  },
  "monotonic_start_time": 1000000,
  "realtime_start_time": 1000000000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "source_node_boot_uuid": "1d782c63-b3c7-466e-bea9-a01308b43333",
  "logger_node_boot_uuid": "1d782c63-b3c7-466e-bea9-a01308b43333",
  "boot_uuids": [
    "1d782c63-b3c7-466e-bea9-a01308b43333",
    "",
    ""
  ],
  "parts_uuid": "bafe9f8e-7dea-4bd9-95f5-3d8390e49208",
  "parts_index": 0
})")),
        config2_(MakeHeader(config_,
                            R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi2"
  },
  "logger_node": {
    "name": "pi2"
  },
  "monotonic_start_time": 0,
  "realtime_start_time": 1000000000000,
  "source_node_boot_uuid": "6f4269ec-547f-4a1a-8281-37aca7fe5dc2",
  "logger_node_boot_uuid": "6f4269ec-547f-4a1a-8281-37aca7fe5dc2",
  "boot_uuids": [
    "",
    "6f4269ec-547f-4a1a-8281-37aca7fe5dc2",
    ""
  ],
  "log_event_uuid": "cb89a1ce-c4b6-4747-a647-051f09ac888c",
  "parts_uuid": "e6bff6c6-757f-4675-90d8-3bfb642870e6",
  "parts_index": 0
})")),
        config3_(MakeHeader(config_,
                            R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi1"
  },
  "logger_node": {
    "name": "pi1"
  },
  "monotonic_start_time": 2000000,
  "realtime_start_time": 1000000000,
  "log_event_uuid": "cb26b86a-473e-4f74-8403-50eb92ed60ad",
  "source_node_boot_uuid": "1d782c63-b3c7-466e-bea9-a01308b43333",
  "logger_node_boot_uuid": "1d782c63-b3c7-466e-bea9-a01308b43333",
  "boot_uuids": [
    "1d782c63-b3c7-466e-bea9-a01308b43333",
    "",
    ""
  ],
  "parts_uuid": "1f098701-949f-4392-81f9-be463e2d7bd4",
  "parts_index": 0
})")),
        config4_(MakeHeader(config_,
                            R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi2"
  },
  "logger_node": {
    "name": "pi1"
  },
  "monotonic_start_time": 2000000,
  "realtime_start_time": 1000000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "parts_uuid": "4e560a47-e2a6-4ce3-a925-490bebc947c5",
  "source_node_boot_uuid": "6f4269ec-547f-4a1a-8281-37aca7fe5dc2",
  "logger_node_boot_uuid": "1d782c63-b3c7-466e-bea9-a01308b43333",
  "boot_uuids": [
    "1d782c63-b3c7-466e-bea9-a01308b43333",
    "6f4269ec-547f-4a1a-8281-37aca7fe5dc2",
    ""
  ],
  "parts_index": 0
})")) {
    unlink(logfile0_.c_str());
    unlink(logfile1_.c_str());
    unlink(logfile2_.c_str());
    unlink(logfile3_.c_str());
    queue_index_.resize(config_.message().channels()->size());
  }

 protected:
  flatbuffers::DetachedBuffer MakeLogMessage(
      const aos::monotonic_clock::time_point monotonic_now, int channel_index,
      int value) {
    flatbuffers::FlatBufferBuilder message_fbb;
    message_fbb.ForceDefaults(true);
    TestMessage::Builder test_message_builder(message_fbb);
    test_message_builder.add_value(value);
    message_fbb.Finish(test_message_builder.Finish());

    aos::Context context;
    context.monotonic_event_time = monotonic_now;
    context.realtime_event_time = aos::realtime_clock::epoch() +
                                  chrono::seconds(1000) +
                                  monotonic_now.time_since_epoch();
    CHECK_LT(static_cast<size_t>(channel_index), queue_index_.size());
    context.queue_index = queue_index_[channel_index];
    context.size = message_fbb.GetSize();
    context.data = message_fbb.GetBufferPointer();

    ++queue_index_[channel_index];

    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(true);
    fbb.FinishSizePrefixed(
        PackMessage(&fbb, context, channel_index, LogType::kLogMessage));

    return fbb.Release();
  }

  flatbuffers::DetachedBuffer MakeTimestampMessage(
      const aos::monotonic_clock::time_point sender_monotonic_now,
      int channel_index, chrono::nanoseconds receiver_monotonic_offset,
      monotonic_clock::time_point monotonic_timestamp_time =
          monotonic_clock::min_time) {
    const monotonic_clock::time_point monotonic_sent_time =
        sender_monotonic_now + receiver_monotonic_offset;

    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(true);

    logger::MessageHeader::Builder message_header_builder(fbb);

    message_header_builder.add_channel_index(channel_index);

    message_header_builder.add_queue_index(queue_index_[channel_index] - 1 +
                                           100);
    message_header_builder.add_monotonic_sent_time(
        monotonic_sent_time.time_since_epoch().count());
    message_header_builder.add_realtime_sent_time(
        (aos::realtime_clock::epoch() + chrono::seconds(1000) +
         monotonic_sent_time.time_since_epoch())
            .time_since_epoch()
            .count());

    message_header_builder.add_monotonic_remote_time(
        sender_monotonic_now.time_since_epoch().count());
    message_header_builder.add_realtime_remote_time(
        (aos::realtime_clock::epoch() + chrono::seconds(1000) +
         sender_monotonic_now.time_since_epoch())
            .time_since_epoch()
            .count());
    message_header_builder.add_remote_queue_index(queue_index_[channel_index] -
                                                  1);

    if (monotonic_timestamp_time != monotonic_clock::min_time) {
      message_header_builder.add_monotonic_timestamp_time(
          monotonic_timestamp_time.time_since_epoch().count());
    }

    fbb.FinishSizePrefixed(message_header_builder.Finish());
    LOG(INFO) << aos::FlatbufferToJson(
        aos::SizePrefixedFlatbufferSpan<MessageHeader>(
            absl::Span<uint8_t>(fbb.GetBufferPointer(), fbb.GetSize())));

    return fbb.Release();
  }

  const std::string logfile0_ = aos::testing::TestTmpDir() + "/log0.bfbs";
  const std::string logfile1_ = aos::testing::TestTmpDir() + "/log1.bfbs";
  const std::string logfile2_ = aos::testing::TestTmpDir() + "/log2.bfbs";
  const std::string logfile3_ = aos::testing::TestTmpDir() + "/log3.bfbs";

  const aos::FlatbufferDetachedBuffer<Configuration> config_;
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> config0_;
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> config1_;
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> config2_;
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> config3_;
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> config4_;

  std::vector<uint32_t> queue_index_;
};

using MessageSorterTest = SortingElementTest;
using MessageSorterDeathTest = MessageSorterTest;
using PartsMergerTest = SortingElementTest;
using TimestampMapperTest = SortingElementTest;

// Tests that we can pull messages out of a log sorted in order.
TEST_F(MessageSorterTest, Pull) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer(logfile0_);
    writer.QueueSpan(config0_.span());
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005));
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 1, 0x105));
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x006));
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1901), 1, 0x107));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_});

  MessageSorter message_sorter(parts[0].parts[0]);

  // Confirm we aren't sorted until any time until the message is popped.
  // Peeking shouldn't change the sorted until time.
  EXPECT_EQ(message_sorter.sorted_until(), monotonic_clock::min_time);

  std::deque<Message> output;

  ASSERT_TRUE(message_sorter.Front() != nullptr);
  output.emplace_back(std::move(*message_sorter.Front()));
  message_sorter.PopFront();
  EXPECT_EQ(message_sorter.sorted_until(), e + chrono::milliseconds(1900));

  ASSERT_TRUE(message_sorter.Front() != nullptr);
  output.emplace_back(std::move(*message_sorter.Front()));
  message_sorter.PopFront();
  EXPECT_EQ(message_sorter.sorted_until(), e + chrono::milliseconds(1900));

  ASSERT_TRUE(message_sorter.Front() != nullptr);
  output.emplace_back(std::move(*message_sorter.Front()));
  message_sorter.PopFront();
  EXPECT_EQ(message_sorter.sorted_until(), monotonic_clock::max_time);

  ASSERT_TRUE(message_sorter.Front() != nullptr);
  output.emplace_back(std::move(*message_sorter.Front()));
  message_sorter.PopFront();
  EXPECT_EQ(message_sorter.sorted_until(), monotonic_clock::max_time);

  ASSERT_TRUE(message_sorter.Front() == nullptr);

  EXPECT_EQ(output[0].timestamp.boot, 0);
  EXPECT_EQ(output[0].timestamp.time, e + chrono::milliseconds(1000));
  EXPECT_EQ(output[1].timestamp.boot, 0);
  EXPECT_EQ(output[1].timestamp.time, e + chrono::milliseconds(1000));
  EXPECT_EQ(output[2].timestamp.boot, 0);
  EXPECT_EQ(output[2].timestamp.time, e + chrono::milliseconds(1901));
  EXPECT_EQ(output[3].timestamp.boot, 0);
  EXPECT_EQ(output[3].timestamp.time, e + chrono::milliseconds(2000));
}

// Tests that we can pull messages out of a log sorted in order.
TEST_F(MessageSorterTest, WayBeforeStart) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer(logfile0_);
    writer.QueueSpan(config0_.span());
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e - chrono::milliseconds(500), 0, 0x005));
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e - chrono::milliseconds(10), 2, 0x005));
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e - chrono::milliseconds(1000), 1, 0x105));
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x006));
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1901), 1, 0x107));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_});

  MessageSorter message_sorter(parts[0].parts[0]);

  // Confirm we aren't sorted until any time until the message is popped.
  // Peeking shouldn't change the sorted until time.
  EXPECT_EQ(message_sorter.sorted_until(), monotonic_clock::min_time);

  std::deque<Message> output;

  for (monotonic_clock::time_point t :
       {e + chrono::milliseconds(1900), e + chrono::milliseconds(1900),
        e + chrono::milliseconds(1900), monotonic_clock::max_time,
        monotonic_clock::max_time}) {
    ASSERT_TRUE(message_sorter.Front() != nullptr);
    output.emplace_back(std::move(*message_sorter.Front()));
    message_sorter.PopFront();
    EXPECT_EQ(message_sorter.sorted_until(), t);
  }

  ASSERT_TRUE(message_sorter.Front() == nullptr);

  EXPECT_EQ(output[0].timestamp.boot, 0u);
  EXPECT_EQ(output[0].timestamp.time, e - chrono::milliseconds(1000));
  EXPECT_EQ(output[1].timestamp.boot, 0u);
  EXPECT_EQ(output[1].timestamp.time, e - chrono::milliseconds(500));
  EXPECT_EQ(output[2].timestamp.boot, 0u);
  EXPECT_EQ(output[2].timestamp.time, e - chrono::milliseconds(10));
  EXPECT_EQ(output[3].timestamp.boot, 0u);
  EXPECT_EQ(output[3].timestamp.time, e + chrono::milliseconds(1901));
  EXPECT_EQ(output[4].timestamp.boot, 0u);
  EXPECT_EQ(output[4].timestamp.time, e + chrono::milliseconds(2000));
}

// Tests that messages too far out of order trigger death.
TEST_F(MessageSorterDeathTest, Pull) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer(logfile0_);
    writer.QueueSpan(config0_.span());
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005));
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 1, 0x105));
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2001), 0, 0x006));
    // The following message is too far out of order and will trigger the CHECK.
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1900), 1, 0x107));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_});

  MessageSorter message_sorter(parts[0].parts[0]);

  // Confirm we aren't sorted until any time until the message is popped.
  // Peeking shouldn't change the sorted until time.
  EXPECT_EQ(message_sorter.sorted_until(), monotonic_clock::min_time);
  std::deque<Message> output;

  ASSERT_TRUE(message_sorter.Front() != nullptr);
  message_sorter.PopFront();
  ASSERT_TRUE(message_sorter.Front() != nullptr);
  ASSERT_TRUE(message_sorter.Front() != nullptr);
  message_sorter.PopFront();

  EXPECT_DEATH({ message_sorter.Front(); },
               "Max out of order of 100000000ns exceeded.");
}

// Tests that we can merge data from 2 separate files, including duplicate data.
TEST_F(PartsMergerTest, TwoFileMerger) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0(logfile0_);
    writer0.QueueSpan(config0_.span());
    TestDetachedBufferWriter writer1(logfile1_);
    writer1.QueueSpan(config1_.span());

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005));
    writer1.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1001), 1, 0x105));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x006));
    writer1.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1002), 1, 0x106));

    // Make a duplicate!
    SizePrefixedFlatbufferDetachedBuffer<MessageHeader> msg(
        MakeLogMessage(e + chrono::milliseconds(3000), 0, 0x007));
    writer0.QueueSpan(msg.span());
    writer1.QueueSpan(msg.span());

    writer1.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(3002), 1, 0x107));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_, logfile1_});
  ASSERT_EQ(parts.size(), 1u);

  PartsMerger merger(FilterPartsForNode(parts, "pi1"));

  EXPECT_EQ(merger.sorted_until(), monotonic_clock::min_time);

  std::deque<Message> output;

  EXPECT_EQ(merger.sorted_until(), monotonic_clock::min_time);
  ASSERT_TRUE(merger.Front() != nullptr);
  EXPECT_EQ(merger.sorted_until(), e + chrono::milliseconds(1900));

  output.emplace_back(std::move(*merger.Front()));
  merger.PopFront();
  EXPECT_EQ(merger.sorted_until(), e + chrono::milliseconds(1900));

  ASSERT_TRUE(merger.Front() != nullptr);
  output.emplace_back(std::move(*merger.Front()));
  merger.PopFront();
  EXPECT_EQ(merger.sorted_until(), e + chrono::milliseconds(2900));

  ASSERT_TRUE(merger.Front() != nullptr);
  output.emplace_back(std::move(*merger.Front()));
  merger.PopFront();
  EXPECT_EQ(merger.sorted_until(), e + chrono::milliseconds(2900));

  ASSERT_TRUE(merger.Front() != nullptr);
  output.emplace_back(std::move(*merger.Front()));
  merger.PopFront();
  EXPECT_EQ(merger.sorted_until(), e + chrono::milliseconds(2900));

  ASSERT_TRUE(merger.Front() != nullptr);
  output.emplace_back(std::move(*merger.Front()));
  merger.PopFront();
  EXPECT_EQ(merger.sorted_until(), monotonic_clock::max_time);

  ASSERT_TRUE(merger.Front() != nullptr);
  output.emplace_back(std::move(*merger.Front()));
  merger.PopFront();
  EXPECT_EQ(merger.sorted_until(), monotonic_clock::max_time);

  ASSERT_TRUE(merger.Front() == nullptr);

  EXPECT_EQ(output[0].timestamp.boot, 0u);
  EXPECT_EQ(output[0].timestamp.time, e + chrono::milliseconds(1000));
  EXPECT_EQ(output[1].timestamp.boot, 0u);
  EXPECT_EQ(output[1].timestamp.time, e + chrono::milliseconds(1001));
  EXPECT_EQ(output[2].timestamp.boot, 0u);
  EXPECT_EQ(output[2].timestamp.time, e + chrono::milliseconds(1002));
  EXPECT_EQ(output[3].timestamp.boot, 0u);
  EXPECT_EQ(output[3].timestamp.time, e + chrono::milliseconds(2000));
  EXPECT_EQ(output[4].timestamp.boot, 0u);
  EXPECT_EQ(output[4].timestamp.time, e + chrono::milliseconds(3000));
  EXPECT_EQ(output[5].timestamp.boot, 0u);
  EXPECT_EQ(output[5].timestamp.time, e + chrono::milliseconds(3002));
}

// Tests that we can merge timestamps with various combinations of
// monotonic_timestamp_time.
TEST_F(PartsMergerTest, TwoFileTimestampMerger) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0(logfile0_);
    writer0.QueueSpan(config0_.span());
    TestDetachedBufferWriter writer1(logfile1_);
    writer1.QueueSpan(config1_.span());

    // Neither has it.
    MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005);
    writer0.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(100)));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(100)));

    // First only has it.
    MakeLogMessage(e + chrono::milliseconds(1001), 0, 0x006);
    writer0.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1001), 0, chrono::seconds(100),
        e + chrono::nanoseconds(971)));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1001), 0, chrono::seconds(100)));

    // Second only has it.
    MakeLogMessage(e + chrono::milliseconds(1002), 0, 0x007);
    writer0.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1002), 0, chrono::seconds(100)));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1002), 0, chrono::seconds(100),
        e + chrono::nanoseconds(972)));

    // Both have it.
    MakeLogMessage(e + chrono::milliseconds(1003), 0, 0x008);
    writer0.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1003), 0, chrono::seconds(100),
        e + chrono::nanoseconds(973)));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1003), 0, chrono::seconds(100),
        e + chrono::nanoseconds(973)));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_, logfile1_});
  ASSERT_EQ(parts.size(), 1u);

  PartsMerger merger(FilterPartsForNode(parts, "pi1"));

  EXPECT_EQ(merger.sorted_until(), monotonic_clock::min_time);

  std::deque<Message> output;

  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(merger.Front() != nullptr);
    output.emplace_back(std::move(*merger.Front()));
    merger.PopFront();
  }
  ASSERT_TRUE(merger.Front() == nullptr);

  EXPECT_EQ(output[0].timestamp.boot, 0u);
  EXPECT_EQ(output[0].timestamp.time, e + chrono::milliseconds(101000));
  EXPECT_FALSE(output[0].data->has_monotonic_timestamp_time);

  EXPECT_EQ(output[1].timestamp.boot, 0u);
  EXPECT_EQ(output[1].timestamp.time, e + chrono::milliseconds(101001));
  EXPECT_TRUE(output[1].data->has_monotonic_timestamp_time);
  EXPECT_EQ(output[1].data->monotonic_timestamp_time,
            monotonic_clock::time_point(std::chrono::nanoseconds(971)));

  EXPECT_EQ(output[2].timestamp.boot, 0u);
  EXPECT_EQ(output[2].timestamp.time, e + chrono::milliseconds(101002));
  EXPECT_TRUE(output[2].data->has_monotonic_timestamp_time);
  EXPECT_EQ(output[2].data->monotonic_timestamp_time,
            monotonic_clock::time_point(std::chrono::nanoseconds(972)));

  EXPECT_EQ(output[3].timestamp.boot, 0u);
  EXPECT_EQ(output[3].timestamp.time, e + chrono::milliseconds(101003));
  EXPECT_TRUE(output[3].data->has_monotonic_timestamp_time);
  EXPECT_EQ(output[3].data->monotonic_timestamp_time,
            monotonic_clock::time_point(std::chrono::nanoseconds(973)));
}

// Tests that we can match timestamps on delivered messages.
TEST_F(TimestampMapperTest, ReadNode0First) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0(logfile0_);
    writer0.QueueSpan(config0_.span());
    TestDetachedBufferWriter writer1(logfile1_);
    writer1.QueueSpan(config2_.span());

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x006));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(2000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(3000), 0, 0x007));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(3000), 0, chrono::seconds(100)));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_, logfile1_});

  ASSERT_EQ(parts[0].logger_node, "pi1");
  ASSERT_EQ(parts[1].logger_node, "pi2");

  size_t mapper0_count = 0;
  TimestampMapper mapper0(FilterPartsForNode(parts, "pi1"));
  mapper0.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper0_count; });
  size_t mapper1_count = 0;
  TimestampMapper mapper1(FilterPartsForNode(parts, "pi2"));
  mapper1.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper1_count; });

  mapper0.AddPeer(&mapper1);
  mapper1.AddPeer(&mapper0);

  {
    std::deque<TimestampedMessage> output0;

    EXPECT_EQ(mapper0_count, 0u);
    EXPECT_EQ(mapper1_count, 0u);
    ASSERT_TRUE(mapper0.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 1u);
    EXPECT_EQ(mapper1_count, 0u);
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    EXPECT_TRUE(mapper0.started());
    EXPECT_EQ(mapper0_count, 1u);
    EXPECT_EQ(mapper1_count, 0u);

    ASSERT_TRUE(mapper0.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 2u);
    EXPECT_EQ(mapper1_count, 0u);
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    EXPECT_TRUE(mapper0.started());

    ASSERT_TRUE(mapper0.Front() != nullptr);
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    EXPECT_TRUE(mapper0.started());

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 0u);

    ASSERT_TRUE(mapper0.Front() == nullptr);

    EXPECT_EQ(output0[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[0].monotonic_event_time.time,
              e + chrono::milliseconds(1000));
    EXPECT_TRUE(output0[0].data != nullptr);

    EXPECT_EQ(output0[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[1].monotonic_event_time.time,
              e + chrono::milliseconds(2000));
    EXPECT_TRUE(output0[1].data != nullptr);

    EXPECT_EQ(output0[2].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[2].monotonic_event_time.time,
              e + chrono::milliseconds(3000));
    EXPECT_TRUE(output0[2].data != nullptr);
  }

  {
    SCOPED_TRACE("Trying node1 now");
    std::deque<TimestampedMessage> output1;

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 0u);

    ASSERT_TRUE(mapper1.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 1u);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 1u);

    ASSERT_TRUE(mapper1.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 2u);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 3u);

    ASSERT_TRUE(mapper1.Front() == nullptr);

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 3u);

    EXPECT_EQ(output1[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1000));
    EXPECT_TRUE(output1[0].data != nullptr);

    EXPECT_EQ(output1[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[1].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(2000));
    EXPECT_TRUE(output1[1].data != nullptr);

    EXPECT_EQ(output1[2].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[2].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(3000));
    EXPECT_TRUE(output1[2].data != nullptr);
  }
}

// Tests that we filter messages using the channel filter callback
TEST_F(TimestampMapperTest, ReplayChannelsCallbackTest) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0(logfile0_);
    writer0.QueueSpan(config0_.span());
    TestDetachedBufferWriter writer1(logfile1_);
    writer1.QueueSpan(config2_.span());

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x006));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(2000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(3000), 0, 0x007));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(3000), 0, chrono::seconds(100)));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_, logfile1_});

  ASSERT_EQ(parts[0].logger_node, "pi1");
  ASSERT_EQ(parts[1].logger_node, "pi2");

  // mapper0 will not provide any messages while mapper1 will provide all
  // messages due to the channel filter callbacks used
  size_t mapper0_count = 0;
  TimestampMapper mapper0(FilterPartsForNode(parts, "pi1"));
  mapper0.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper0_count; });
  mapper0.set_replay_channels_callback(
      [&](const TimestampedMessage &) -> bool { return mapper0_count != 2; });
  size_t mapper1_count = 0;
  TimestampMapper mapper1(FilterPartsForNode(parts, "pi2"));
  mapper1.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper1_count; });
  mapper1.set_replay_channels_callback(
      [&](const TimestampedMessage &) -> bool { return mapper1_count != 2; });

  mapper0.AddPeer(&mapper1);
  mapper1.AddPeer(&mapper0);

  {
    std::deque<TimestampedMessage> output0;

    EXPECT_EQ(mapper0_count, 0u);
    EXPECT_EQ(mapper1_count, 0u);

    ASSERT_TRUE(mapper0.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 1u);
    EXPECT_EQ(mapper1_count, 0u);
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();

    EXPECT_TRUE(mapper0.started());
    EXPECT_EQ(mapper0_count, 1u);
    EXPECT_EQ(mapper1_count, 0u);

    // mapper0_count is now at 3 since the second message is not queued, but
    // timestamp_callback needs to be called everytime even if Front() does not
    // provide a message due to the replay_channels_callback.
    ASSERT_TRUE(mapper0.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 0u);
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();

    EXPECT_TRUE(mapper0.started());
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 0u);

    ASSERT_TRUE(mapper0.Front() == nullptr);
    EXPECT_TRUE(mapper0.started());

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 0u);

    EXPECT_EQ(output0[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[0].monotonic_event_time.time,
              e + chrono::milliseconds(1000));
    EXPECT_TRUE(output0[0].data != nullptr);

    EXPECT_EQ(output0[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[1].monotonic_event_time.time,
              e + chrono::milliseconds(3000));
    EXPECT_TRUE(output0[1].data != nullptr);
  }

  {
    SCOPED_TRACE("Trying node1 now");
    std::deque<TimestampedMessage> output1;

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 0u);

    ASSERT_TRUE(mapper1.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 1u);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 1u);

    // mapper1_count is now at 3 since the second message is not queued, but
    // timestamp_callback needs to be called everytime even if Front() does not
    // provide a message due to the replay_channels_callback.
    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 3u);

    ASSERT_TRUE(mapper1.Front() == nullptr);

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 3u);

    EXPECT_EQ(output1[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1000));
    EXPECT_TRUE(output1[0].data != nullptr);

    EXPECT_EQ(output1[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[1].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(3000));
    EXPECT_TRUE(output1[1].data != nullptr);
  }
}
// Tests that a MessageHeader with monotonic_timestamp_time set gets properly
// returned.
TEST_F(TimestampMapperTest, MessageWithTimestampTime) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0(logfile0_);
    writer0.QueueSpan(config0_.span());
    TestDetachedBufferWriter writer1(logfile1_);
    writer1.QueueSpan(config4_.span());

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(100),
        e + chrono::nanoseconds(971)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x006));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(2000), 0, chrono::seconds(100),
        e + chrono::nanoseconds(5458)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(3000), 0, 0x007));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(3000), 0, chrono::seconds(100)));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_, logfile1_});

  for (const auto &p : parts) {
    LOG(INFO) << p;
  }

  ASSERT_EQ(parts.size(), 1u);

  size_t mapper0_count = 0;
  TimestampMapper mapper0(FilterPartsForNode(parts, "pi1"));
  mapper0.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper0_count; });
  size_t mapper1_count = 0;
  TimestampMapper mapper1(FilterPartsForNode(parts, "pi2"));
  mapper1.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper1_count; });

  mapper0.AddPeer(&mapper1);
  mapper1.AddPeer(&mapper0);

  {
    std::deque<TimestampedMessage> output0;

    for (int i = 0; i < 3; ++i) {
      ASSERT_TRUE(mapper0.Front() != nullptr) << ": " << i;
      output0.emplace_back(std::move(*mapper0.Front()));
      mapper0.PopFront();
    }

    ASSERT_TRUE(mapper0.Front() == nullptr);

    EXPECT_EQ(output0[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[0].monotonic_event_time.time,
              e + chrono::milliseconds(1000));
    EXPECT_EQ(output0[0].monotonic_timestamp_time.boot, 0u);
    EXPECT_EQ(output0[0].monotonic_timestamp_time.time,
              monotonic_clock::min_time);
    EXPECT_TRUE(output0[0].data != nullptr);

    EXPECT_EQ(output0[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[1].monotonic_event_time.time,
              e + chrono::milliseconds(2000));
    EXPECT_EQ(output0[1].monotonic_timestamp_time.boot, 0u);
    EXPECT_EQ(output0[1].monotonic_timestamp_time.time,
              monotonic_clock::min_time);
    EXPECT_TRUE(output0[1].data != nullptr);

    EXPECT_EQ(output0[2].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[2].monotonic_event_time.time,
              e + chrono::milliseconds(3000));
    EXPECT_EQ(output0[2].monotonic_timestamp_time.boot, 0u);
    EXPECT_EQ(output0[2].monotonic_timestamp_time.time,
              monotonic_clock::min_time);
    EXPECT_TRUE(output0[2].data != nullptr);
  }

  {
    SCOPED_TRACE("Trying node1 now");
    std::deque<TimestampedMessage> output1;

    for (int i = 0; i < 3; ++i) {
      ASSERT_TRUE(mapper1.Front() != nullptr);
      output1.emplace_back(std::move(*mapper1.Front()));
      mapper1.PopFront();
    }

    ASSERT_TRUE(mapper1.Front() == nullptr);

    EXPECT_EQ(output1[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1000));
    EXPECT_EQ(output1[0].monotonic_timestamp_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_timestamp_time.time,
              e + chrono::nanoseconds(971));
    EXPECT_TRUE(output1[0].data != nullptr);

    EXPECT_EQ(output1[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[1].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(2000));
    EXPECT_EQ(output1[1].monotonic_timestamp_time.boot, 0u);
    EXPECT_EQ(output1[1].monotonic_timestamp_time.time,
              e + chrono::nanoseconds(5458));
    EXPECT_TRUE(output1[1].data != nullptr);

    EXPECT_EQ(output1[2].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[2].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(3000));
    EXPECT_EQ(output1[2].monotonic_timestamp_time.boot, 0u);
    EXPECT_EQ(output1[2].monotonic_timestamp_time.time,
              monotonic_clock::min_time);
    EXPECT_TRUE(output1[2].data != nullptr);
  }

  EXPECT_EQ(mapper0_count, 3u);
  EXPECT_EQ(mapper1_count, 3u);
}

// Tests that we can match timestamps on delivered messages.  By doing this in
// the reverse order, the second node needs to queue data up from the first node
// to find the matching timestamp.
TEST_F(TimestampMapperTest, ReadNode1First) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0(logfile0_);
    writer0.QueueSpan(config0_.span());
    TestDetachedBufferWriter writer1(logfile1_);
    writer1.QueueSpan(config2_.span());

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x006));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(2000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(3000), 0, 0x007));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(3000), 0, chrono::seconds(100)));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_, logfile1_});

  ASSERT_EQ(parts[0].logger_node, "pi1");
  ASSERT_EQ(parts[1].logger_node, "pi2");

  size_t mapper0_count = 0;
  TimestampMapper mapper0(FilterPartsForNode(parts, "pi1"));
  mapper0.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper0_count; });
  size_t mapper1_count = 0;
  TimestampMapper mapper1(FilterPartsForNode(parts, "pi2"));
  mapper1.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper1_count; });

  mapper0.AddPeer(&mapper1);
  mapper1.AddPeer(&mapper0);

  {
    SCOPED_TRACE("Trying node1 now");
    std::deque<TimestampedMessage> output1;

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    ASSERT_TRUE(mapper1.Front() == nullptr);

    EXPECT_EQ(output1[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1000));
    EXPECT_TRUE(output1[0].data != nullptr);

    EXPECT_EQ(output1[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[1].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(2000));
    EXPECT_TRUE(output1[1].data != nullptr);

    EXPECT_EQ(output1[2].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[2].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(3000));
    EXPECT_TRUE(output1[2].data != nullptr);
  }

  {
    std::deque<TimestampedMessage> output0;

    ASSERT_TRUE(mapper0.Front() != nullptr);
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    EXPECT_TRUE(mapper0.started());

    ASSERT_TRUE(mapper0.Front() != nullptr);
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    EXPECT_TRUE(mapper0.started());

    ASSERT_TRUE(mapper0.Front() != nullptr);
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    EXPECT_TRUE(mapper0.started());

    ASSERT_TRUE(mapper0.Front() == nullptr);

    EXPECT_EQ(output0[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[0].monotonic_event_time.time,
              e + chrono::milliseconds(1000));
    EXPECT_TRUE(output0[0].data != nullptr);

    EXPECT_EQ(output0[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[1].monotonic_event_time.time,
              e + chrono::milliseconds(2000));
    EXPECT_TRUE(output0[1].data != nullptr);

    EXPECT_EQ(output0[2].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[2].monotonic_event_time.time,
              e + chrono::milliseconds(3000));
    EXPECT_TRUE(output0[2].data != nullptr);
  }

  EXPECT_EQ(mapper0_count, 3u);
  EXPECT_EQ(mapper1_count, 3u);
}

// Tests that we return just the timestamps if we couldn't find the data and the
// missing data was at the beginning of the file.
TEST_F(TimestampMapperTest, ReadMissingDataBefore) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0(logfile0_);
    writer0.QueueSpan(config0_.span());
    TestDetachedBufferWriter writer1(logfile1_);
    writer1.QueueSpan(config2_.span());

    MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005);
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x006));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(2000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(3000), 0, 0x007));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(3000), 0, chrono::seconds(100)));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_, logfile1_});

  ASSERT_EQ(parts[0].logger_node, "pi1");
  ASSERT_EQ(parts[1].logger_node, "pi2");

  size_t mapper0_count = 0;
  TimestampMapper mapper0(FilterPartsForNode(parts, "pi1"));
  mapper0.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper0_count; });
  size_t mapper1_count = 0;
  TimestampMapper mapper1(FilterPartsForNode(parts, "pi2"));
  mapper1.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper1_count; });

  mapper0.AddPeer(&mapper1);
  mapper1.AddPeer(&mapper0);

  {
    SCOPED_TRACE("Trying node1 now");
    std::deque<TimestampedMessage> output1;

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    ASSERT_TRUE(mapper1.Front() == nullptr);

    EXPECT_EQ(output1[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1000));
    EXPECT_FALSE(output1[0].data != nullptr);

    EXPECT_EQ(output1[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[1].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(2000));
    EXPECT_TRUE(output1[1].data != nullptr);

    EXPECT_EQ(output1[2].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[2].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(3000));
    EXPECT_TRUE(output1[2].data != nullptr);
  }

  EXPECT_EQ(mapper0_count, 0u);
  EXPECT_EQ(mapper1_count, 3u);
}

// Tests that we return just the timestamps if we couldn't find the data and the
// missing data was at the end of the file.
TEST_F(TimestampMapperTest, ReadMissingDataAfter) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0(logfile0_);
    writer0.QueueSpan(config0_.span());
    TestDetachedBufferWriter writer1(logfile1_);
    writer1.QueueSpan(config2_.span());

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x006));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(2000), 0, chrono::seconds(100)));

    MakeLogMessage(e + chrono::milliseconds(3000), 0, 0x007);
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(3000), 0, chrono::seconds(100)));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_, logfile1_});

  ASSERT_EQ(parts[0].logger_node, "pi1");
  ASSERT_EQ(parts[1].logger_node, "pi2");

  size_t mapper0_count = 0;
  TimestampMapper mapper0(FilterPartsForNode(parts, "pi1"));
  mapper0.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper0_count; });
  size_t mapper1_count = 0;
  TimestampMapper mapper1(FilterPartsForNode(parts, "pi2"));
  mapper1.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper1_count; });

  mapper0.AddPeer(&mapper1);
  mapper1.AddPeer(&mapper0);

  {
    SCOPED_TRACE("Trying node1 now");
    std::deque<TimestampedMessage> output1;

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    ASSERT_TRUE(mapper1.Front() == nullptr);

    EXPECT_EQ(output1[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1000));
    EXPECT_TRUE(output1[0].data != nullptr);

    EXPECT_EQ(output1[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[1].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(2000));
    EXPECT_TRUE(output1[1].data != nullptr);

    EXPECT_EQ(output1[2].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[2].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(3000));
    EXPECT_FALSE(output1[2].data != nullptr);
  }

  EXPECT_EQ(mapper0_count, 0u);
  EXPECT_EQ(mapper1_count, 3u);
}

// Tests that we handle a message which failed to forward or be logged.
TEST_F(TimestampMapperTest, ReadMissingDataMiddle) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0(logfile0_);
    writer0.QueueSpan(config0_.span());
    TestDetachedBufferWriter writer1(logfile1_);
    writer1.QueueSpan(config2_.span());

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(100)));

    // Create both the timestamp and message, but don't log them, simulating a
    // forwarding drop.
    MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x006);
    MakeTimestampMessage(e + chrono::milliseconds(2000), 0,
                         chrono::seconds(100));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(3000), 0, 0x007));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(3000), 0, chrono::seconds(100)));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_, logfile1_});

  ASSERT_EQ(parts[0].logger_node, "pi1");
  ASSERT_EQ(parts[1].logger_node, "pi2");

  size_t mapper0_count = 0;
  TimestampMapper mapper0(FilterPartsForNode(parts, "pi1"));
  mapper0.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper0_count; });
  size_t mapper1_count = 0;
  TimestampMapper mapper1(FilterPartsForNode(parts, "pi2"));
  mapper1.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper1_count; });

  mapper0.AddPeer(&mapper1);
  mapper1.AddPeer(&mapper0);

  {
    std::deque<TimestampedMessage> output1;

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));

    ASSERT_FALSE(mapper1.Front() == nullptr);

    EXPECT_EQ(output1[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1000));
    EXPECT_TRUE(output1[0].data != nullptr);

    EXPECT_EQ(output1[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[1].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(3000));
    EXPECT_TRUE(output1[1].data != nullptr);
  }

  EXPECT_EQ(mapper0_count, 0u);
  EXPECT_EQ(mapper1_count, 2u);
}

// Tests that we properly sort log files with duplicate timestamps.
TEST_F(TimestampMapperTest, ReadSameTimestamp) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0(logfile0_);
    writer0.QueueSpan(config0_.span());
    TestDetachedBufferWriter writer1(logfile1_);
    writer1.QueueSpan(config2_.span());

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x006));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(2000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x007));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(2000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(3000), 0, 0x008));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(3000), 0, chrono::seconds(100)));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_, logfile1_});

  ASSERT_EQ(parts[0].logger_node, "pi1");
  ASSERT_EQ(parts[1].logger_node, "pi2");

  size_t mapper0_count = 0;
  TimestampMapper mapper0(FilterPartsForNode(parts, "pi1"));
  mapper0.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper0_count; });
  size_t mapper1_count = 0;
  TimestampMapper mapper1(FilterPartsForNode(parts, "pi2"));
  mapper1.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper1_count; });

  mapper0.AddPeer(&mapper1);
  mapper1.AddPeer(&mapper0);

  {
    SCOPED_TRACE("Trying node1 now");
    std::deque<TimestampedMessage> output1;

    for (int i = 0; i < 4; ++i) {
      ASSERT_TRUE(mapper1.Front() != nullptr);
      output1.emplace_back(std::move(*mapper1.Front()));
      mapper1.PopFront();
    }
    ASSERT_TRUE(mapper1.Front() == nullptr);

    EXPECT_EQ(output1[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1000));
    EXPECT_TRUE(output1[0].data != nullptr);

    EXPECT_EQ(output1[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[1].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(2000));
    EXPECT_TRUE(output1[1].data != nullptr);

    EXPECT_EQ(output1[2].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[2].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(2000));
    EXPECT_TRUE(output1[2].data != nullptr);

    EXPECT_EQ(output1[3].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[3].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(3000));
    EXPECT_TRUE(output1[3].data != nullptr);
  }

  EXPECT_EQ(mapper0_count, 0u);
  EXPECT_EQ(mapper1_count, 4u);
}

// Tests that we properly produce a valid start time.
TEST_F(TimestampMapperTest, StartTime) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0(logfile0_);
    writer0.QueueSpan(config0_.span());
    TestDetachedBufferWriter writer1(logfile1_);
    writer1.QueueSpan(config1_.span());
    TestDetachedBufferWriter writer2(logfile2_);
    writer2.QueueSpan(config3_.span());
  }

  const std::vector<LogFile> parts =
      SortParts({logfile0_, logfile1_, logfile2_});

  size_t mapper0_count = 0;
  TimestampMapper mapper0(FilterPartsForNode(parts, "pi1"));
  mapper0.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper0_count; });

  EXPECT_EQ(mapper0.monotonic_start_time(0), e + chrono::milliseconds(1));
  EXPECT_EQ(mapper0.realtime_start_time(0),
            realtime_clock::time_point(chrono::seconds(1000)));
  EXPECT_EQ(mapper0_count, 0u);
}

// Tests that when a peer isn't registered, we treat that as if there was no
// data available.
TEST_F(TimestampMapperTest, NoPeer) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0(logfile0_);
    writer0.QueueSpan(config0_.span());
    TestDetachedBufferWriter writer1(logfile1_);
    writer1.QueueSpan(config2_.span());

    MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005);
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x006));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(2000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(3000), 0, 0x007));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(3000), 0, chrono::seconds(100)));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_, logfile1_});

  ASSERT_EQ(parts[0].logger_node, "pi1");
  ASSERT_EQ(parts[1].logger_node, "pi2");

  size_t mapper1_count = 0;
  TimestampMapper mapper1(FilterPartsForNode(parts, "pi2"));
  mapper1.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper1_count; });

  {
    std::deque<TimestampedMessage> output1;

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    ASSERT_TRUE(mapper1.Front() == nullptr);

    EXPECT_EQ(output1[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1000));
    EXPECT_FALSE(output1[0].data != nullptr);

    EXPECT_EQ(output1[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[1].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(2000));
    EXPECT_FALSE(output1[1].data != nullptr);

    EXPECT_EQ(output1[2].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[2].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(3000));
    EXPECT_FALSE(output1[2].data != nullptr);
  }
  EXPECT_EQ(mapper1_count, 3u);
}

// Tests that we can queue messages and call the timestamp callback for both
// nodes.
TEST_F(TimestampMapperTest, QueueUntilNode0) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0(logfile0_);
    writer0.QueueSpan(config0_.span());
    TestDetachedBufferWriter writer1(logfile1_);
    writer1.QueueSpan(config2_.span());

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x006));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x007));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(2000), 0, chrono::seconds(100)));

    writer0.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(3000), 0, 0x008));
    writer1.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(3000), 0, chrono::seconds(100)));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_, logfile1_});

  ASSERT_EQ(parts[0].logger_node, "pi1");
  ASSERT_EQ(parts[1].logger_node, "pi2");

  size_t mapper0_count = 0;
  TimestampMapper mapper0(FilterPartsForNode(parts, "pi1"));
  mapper0.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper0_count; });
  size_t mapper1_count = 0;
  TimestampMapper mapper1(FilterPartsForNode(parts, "pi2"));
  mapper1.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper1_count; });

  mapper0.AddPeer(&mapper1);
  mapper1.AddPeer(&mapper0);

  {
    std::deque<TimestampedMessage> output0;

    EXPECT_EQ(mapper0_count, 0u);
    EXPECT_EQ(mapper1_count, 0u);
    mapper0.QueueUntil(
        BootTimestamp{.boot = 0, .time = e + chrono::milliseconds(1000)});
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 0u);

    ASSERT_TRUE(mapper0.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 0u);

    mapper0.QueueUntil(
        BootTimestamp{.boot = 0, .time = e + chrono::milliseconds(1500)});
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 0u);

    mapper0.QueueUntil(
        BootTimestamp{.boot = 0, .time = e + chrono::milliseconds(2500)});
    EXPECT_EQ(mapper0_count, 4u);
    EXPECT_EQ(mapper1_count, 0u);

    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();

    EXPECT_EQ(mapper0_count, 4u);
    EXPECT_EQ(mapper1_count, 0u);

    ASSERT_TRUE(mapper0.Front() == nullptr);

    EXPECT_EQ(output0[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[0].monotonic_event_time.time,
              e + chrono::milliseconds(1000));
    EXPECT_TRUE(output0[0].data != nullptr);

    EXPECT_EQ(output0[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[1].monotonic_event_time.time,
              e + chrono::milliseconds(1000));
    EXPECT_TRUE(output0[1].data != nullptr);

    EXPECT_EQ(output0[2].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[2].monotonic_event_time.time,
              e + chrono::milliseconds(2000));
    EXPECT_TRUE(output0[2].data != nullptr);

    EXPECT_EQ(output0[3].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[3].monotonic_event_time.time,
              e + chrono::milliseconds(3000));
    EXPECT_TRUE(output0[3].data != nullptr);
  }

  {
    SCOPED_TRACE("Trying node1 now");
    std::deque<TimestampedMessage> output1;

    EXPECT_EQ(mapper0_count, 4u);
    EXPECT_EQ(mapper1_count, 0u);
    mapper1.QueueUntil(BootTimestamp{
        .boot = 0,
        .time = e + chrono::seconds(100) + chrono::milliseconds(1000)});
    EXPECT_EQ(mapper0_count, 4u);
    EXPECT_EQ(mapper1_count, 3u);

    ASSERT_TRUE(mapper1.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 4u);
    EXPECT_EQ(mapper1_count, 3u);

    mapper1.QueueUntil(BootTimestamp{
        .boot = 0,
        .time = e + chrono::seconds(100) + chrono::milliseconds(1500)});
    EXPECT_EQ(mapper0_count, 4u);
    EXPECT_EQ(mapper1_count, 3u);

    mapper1.QueueUntil(BootTimestamp{
        .boot = 0,
        .time = e + chrono::seconds(100) + chrono::milliseconds(2500)});
    EXPECT_EQ(mapper0_count, 4u);
    EXPECT_EQ(mapper1_count, 4u);

    ASSERT_TRUE(mapper1.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 4u);
    EXPECT_EQ(mapper1_count, 4u);

    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();

    EXPECT_EQ(mapper0_count, 4u);
    EXPECT_EQ(mapper1_count, 4u);

    ASSERT_TRUE(mapper1.Front() == nullptr);

    EXPECT_EQ(mapper0_count, 4u);
    EXPECT_EQ(mapper1_count, 4u);

    EXPECT_EQ(output1[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1000));
    EXPECT_TRUE(output1[0].data != nullptr);

    EXPECT_EQ(output1[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[1].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1000));
    EXPECT_TRUE(output1[1].data != nullptr);

    EXPECT_EQ(output1[2].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[2].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(2000));
    EXPECT_TRUE(output1[2].data != nullptr);

    EXPECT_EQ(output1[3].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[3].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(3000));
    EXPECT_TRUE(output1[3].data != nullptr);
  }
}

class BootMergerTest : public SortingElementTest {
 public:
  BootMergerTest()
      : SortingElementTest(),
        boot0_(MakeHeader(config_, R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi2"
  },
  "logger_node": {
    "name": "pi1"
  },
  "monotonic_start_time": 1000000,
  "realtime_start_time": 1000000000000,
  "logger_monotonic_start_time": 1000000,
  "logger_realtime_start_time": 1000000000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "parts_uuid": "1a9e5ca2-31b2-475b-8282-88f6d1ce5109",
  "parts_index": 0,
  "logger_instance_uuid": "1c3142ad-10a5-408d-a760-b63b73d3b904",
  "logger_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "source_node_boot_uuid": "6ba4f28d-21a2-4d7f-83f4-ee365cf86464",
  "boot_uuids": [
    "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
    "6ba4f28d-21a2-4d7f-83f4-ee365cf86464",
    ""
  ]
})")),
        boot1_(MakeHeader(config_, R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi2"
  },
  "logger_node": {
    "name": "pi1"
  },
  "monotonic_start_time": 1000000,
  "realtime_start_time": 1000000000000,
  "logger_monotonic_start_time": 1000000,
  "logger_realtime_start_time": 1000000000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "parts_uuid": "2a05d725-5d5c-4c0b-af42-88de2f3c3876",
  "parts_index": 1,
  "logger_instance_uuid": "1c3142ad-10a5-408d-a760-b63b73d3b904",
  "logger_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "source_node_boot_uuid": "b728d27a-9181-4eac-bfc1-5d09b80469d2",
  "boot_uuids": [
    "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
    "b728d27a-9181-4eac-bfc1-5d09b80469d2",
    ""
  ]
})")) {}

 protected:
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> boot0_;
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> boot1_;
};

// This tests that we can properly sort a multi-node log file which has the old
// (and buggy) timestamps in the header, and the non-resetting parts_index.
// These make it so we can just bairly figure out what happened first and what
// happened second, but not in a way that is robust to multiple nodes rebooting.
TEST_F(BootMergerTest, OldReboot) {
  {
    TestDetachedBufferWriter writer(logfile0_);
    writer.QueueSpan(boot0_.span());
  }
  {
    TestDetachedBufferWriter writer(logfile1_);
    writer.QueueSpan(boot1_.span());
  }

  const std::vector<LogFile> parts = SortParts({logfile0_, logfile1_});

  ASSERT_EQ(parts.size(), 1u);
  ASSERT_EQ(parts[0].parts.size(), 2u);

  EXPECT_EQ(parts[0].parts[0].boot_count, 0);
  EXPECT_EQ(parts[0].parts[0].source_boot_uuid,
            boot0_.message().source_node_boot_uuid()->string_view());

  EXPECT_EQ(parts[0].parts[1].boot_count, 1);
  EXPECT_EQ(parts[0].parts[1].source_boot_uuid,
            boot1_.message().source_node_boot_uuid()->string_view());
}

// This tests that we can produce messages ordered across a reboot.
TEST_F(BootMergerTest, SortAcrossReboot) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer(logfile0_);
    writer.QueueSpan(boot0_.span());
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005));
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 1, 0x105));
  }
  {
    TestDetachedBufferWriter writer(logfile1_);
    writer.QueueSpan(boot1_.span());
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(100), 0, 0x006));
    writer.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(200), 1, 0x106));
  }

  const std::vector<LogFile> parts = SortParts({logfile0_, logfile1_});
  ASSERT_EQ(parts.size(), 1u);
  ASSERT_EQ(parts[0].parts.size(), 2u);

  BootMerger merger(FilterPartsForNode(parts, "pi2"));

  EXPECT_EQ(merger.node(), 1u);

  std::vector<Message> output;
  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(merger.Front() != nullptr);
    output.emplace_back(std::move(*merger.Front()));
    merger.PopFront();
  }

  ASSERT_TRUE(merger.Front() == nullptr);

  EXPECT_EQ(output[0].timestamp.boot, 0u);
  EXPECT_EQ(output[0].timestamp.time, e + chrono::milliseconds(1000));
  EXPECT_EQ(output[1].timestamp.boot, 0u);
  EXPECT_EQ(output[1].timestamp.time, e + chrono::milliseconds(2000));

  EXPECT_EQ(output[2].timestamp.boot, 1u);
  EXPECT_EQ(output[2].timestamp.time, e + chrono::milliseconds(100));
  EXPECT_EQ(output[3].timestamp.boot, 1u);
  EXPECT_EQ(output[3].timestamp.time, e + chrono::milliseconds(200));
}

class RebootTimestampMapperTest : public SortingElementTest {
 public:
  RebootTimestampMapperTest()
      : SortingElementTest(),
        boot0a_(MakeHeader(config_, R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi1"
  },
  "logger_node": {
    "name": "pi1"
  },
  "monotonic_start_time": 1000000,
  "realtime_start_time": 1000000000000,
  "logger_monotonic_start_time": 1000000,
  "logger_realtime_start_time": 1000000000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "parts_uuid": "ee4f5a98-77d0-4e01-af2f-bbb29e098ede",
  "parts_index": 0,
  "logger_instance_uuid": "1c3142ad-10a5-408d-a760-b63b73d3b904",
  "logger_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "source_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "boot_uuids": [
    "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
    "6ba4f28d-21a2-4d7f-83f4-ee365cf86464",
    ""
  ]
})")),
        boot0b_(MakeHeader(config_, R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi1"
  },
  "logger_node": {
    "name": "pi1"
  },
  "monotonic_start_time": 1000000,
  "realtime_start_time": 1000000000000,
  "logger_monotonic_start_time": 1000000,
  "logger_realtime_start_time": 1000000000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "parts_uuid": "ee4f5a98-77d0-4e01-af2f-bbb29e098ede",
  "parts_index": 1,
  "logger_instance_uuid": "1c3142ad-10a5-408d-a760-b63b73d3b904",
  "logger_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "source_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "boot_uuids": [
    "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
    "b728d27a-9181-4eac-bfc1-5d09b80469d2",
    ""
  ]
})")),
        boot1a_(MakeHeader(config_, R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi2"
  },
  "logger_node": {
    "name": "pi1"
  },
  "monotonic_start_time": 1000000,
  "realtime_start_time": 1000000000000,
  "logger_monotonic_start_time": 1000000,
  "logger_realtime_start_time": 1000000000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "parts_uuid": "f6ab0cdc-a654-456d-bfd9-2bbc09098edf",
  "parts_index": 0,
  "logger_instance_uuid": "1c3142ad-10a5-408d-a760-b63b73d3b904",
  "logger_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "source_node_boot_uuid": "6ba4f28d-21a2-4d7f-83f4-ee365cf86464",
  "boot_uuids": [
    "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
    "6ba4f28d-21a2-4d7f-83f4-ee365cf86464",
    ""
  ]
})")),
        boot1b_(MakeHeader(config_, R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi2"
  },
  "logger_node": {
    "name": "pi1"
  },
  "monotonic_start_time": 1000000,
  "realtime_start_time": 1000000000000,
  "logger_monotonic_start_time": 1000000,
  "logger_realtime_start_time": 1000000000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "parts_uuid": "3a9d0445-f520-43ca-93f5-e2cc7f54d40a",
  "parts_index": 1,
  "logger_instance_uuid": "1c3142ad-10a5-408d-a760-b63b73d3b904",
  "logger_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "source_node_boot_uuid": "b728d27a-9181-4eac-bfc1-5d09b80469d2",
  "boot_uuids": [
    "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
    "b728d27a-9181-4eac-bfc1-5d09b80469d2",
    ""
  ]
})")) {}

 protected:
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> boot0a_;
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> boot0b_;
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> boot1a_;
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> boot1b_;
};

// Tests that we can match timestamps on delivered messages in the presence of
// reboots on the node receiving timestamps.
TEST_F(RebootTimestampMapperTest, ReadNode0First) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0a(logfile0_);
    writer0a.QueueSpan(boot0a_.span());
    TestDetachedBufferWriter writer0b(logfile1_);
    writer0b.QueueSpan(boot0b_.span());
    TestDetachedBufferWriter writer1a(logfile2_);
    writer1a.QueueSpan(boot1a_.span());
    TestDetachedBufferWriter writer1b(logfile3_);
    writer1b.QueueSpan(boot1b_.span());

    writer0a.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(1000), 0, 0x005));
    writer1a.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(100),
        e + chrono::milliseconds(1001)));

    writer1b.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(1000), 0, chrono::seconds(21),
        e + chrono::milliseconds(2001)));

    writer0b.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(2000), 0, 0x006));
    writer1b.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(2000), 0, chrono::seconds(20),
        e + chrono::milliseconds(2001)));

    writer0b.WriteSizedFlatbuffer(
        MakeLogMessage(e + chrono::milliseconds(3000), 0, 0x007));
    writer1b.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::milliseconds(3000), 0, chrono::seconds(20),
        e + chrono::milliseconds(3001)));
  }

  const std::vector<LogFile> parts =
      SortParts({logfile0_, logfile1_, logfile2_, logfile3_});

  for (const auto &x : parts) {
    LOG(INFO) << x;
  }
  ASSERT_EQ(parts.size(), 1u);
  ASSERT_EQ(parts[0].logger_node, "pi1");

  size_t mapper0_count = 0;
  TimestampMapper mapper0(FilterPartsForNode(parts, "pi1"));
  mapper0.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper0_count; });
  size_t mapper1_count = 0;
  TimestampMapper mapper1(FilterPartsForNode(parts, "pi2"));
  mapper1.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper1_count; });

  mapper0.AddPeer(&mapper1);
  mapper1.AddPeer(&mapper0);

  {
    std::deque<TimestampedMessage> output0;

    EXPECT_EQ(mapper0_count, 0u);
    EXPECT_EQ(mapper1_count, 0u);
    ASSERT_TRUE(mapper0.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 1u);
    EXPECT_EQ(mapper1_count, 0u);
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    EXPECT_TRUE(mapper0.started());
    EXPECT_EQ(mapper0_count, 1u);
    EXPECT_EQ(mapper1_count, 0u);

    ASSERT_TRUE(mapper0.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 2u);
    EXPECT_EQ(mapper1_count, 0u);
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    EXPECT_TRUE(mapper0.started());

    ASSERT_TRUE(mapper0.Front() != nullptr);
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    EXPECT_TRUE(mapper0.started());

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 0u);

    ASSERT_TRUE(mapper0.Front() == nullptr);

    LOG(INFO) << output0[0];
    LOG(INFO) << output0[1];
    LOG(INFO) << output0[2];

    EXPECT_EQ(output0[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[0].monotonic_event_time.time,
              e + chrono::milliseconds(1000));
    EXPECT_EQ(output0[0].queue_index,
              (BootQueueIndex{.boot = 0u, .index = 0u}));
    EXPECT_EQ(output0[0].monotonic_remote_time, BootTimestamp::min_time());
    EXPECT_EQ(output0[0].monotonic_timestamp_time, BootTimestamp::min_time());
    EXPECT_TRUE(output0[0].data != nullptr);

    EXPECT_EQ(output0[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[1].monotonic_event_time.time,
              e + chrono::milliseconds(2000));
    EXPECT_EQ(output0[1].queue_index,
              (BootQueueIndex{.boot = 0u, .index = 1u}));
    EXPECT_EQ(output0[1].monotonic_remote_time, BootTimestamp::min_time());
    EXPECT_EQ(output0[1].monotonic_timestamp_time, BootTimestamp::min_time());
    EXPECT_TRUE(output0[1].data != nullptr);

    EXPECT_EQ(output0[2].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[2].monotonic_event_time.time,
              e + chrono::milliseconds(3000));
    EXPECT_EQ(output0[2].queue_index,
              (BootQueueIndex{.boot = 0u, .index = 2u}));
    EXPECT_EQ(output0[2].monotonic_remote_time, BootTimestamp::min_time());
    EXPECT_EQ(output0[2].monotonic_timestamp_time, BootTimestamp::min_time());
    EXPECT_TRUE(output0[2].data != nullptr);
  }

  {
    SCOPED_TRACE("Trying node1 now");
    std::deque<TimestampedMessage> output1;

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 0u);

    ASSERT_TRUE(mapper1.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 1u);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 1u);

    ASSERT_TRUE(mapper1.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 2u);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 4u);

    ASSERT_TRUE(mapper1.Front() == nullptr);

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 4u);

    EXPECT_EQ(output1[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1000));
    EXPECT_EQ(output1[0].monotonic_remote_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_remote_time.time,
              e + chrono::milliseconds(1000));
    EXPECT_EQ(output1[0].remote_queue_index,
              (BootQueueIndex{.boot = 0u, .index = 0u}));
    EXPECT_EQ(output1[0].monotonic_timestamp_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_timestamp_time.time,
              e + chrono::milliseconds(1001));
    EXPECT_TRUE(output1[0].data != nullptr);

    EXPECT_EQ(output1[1].monotonic_event_time.boot, 1u);
    EXPECT_EQ(output1[1].monotonic_event_time.time,
              e + chrono::seconds(20) + chrono::milliseconds(2000));
    EXPECT_EQ(output1[1].remote_queue_index,
              (BootQueueIndex{.boot = 0u, .index = 0u}));
    EXPECT_EQ(output1[1].monotonic_remote_time.boot, 0u);
    EXPECT_EQ(output1[1].monotonic_remote_time.time,
              e + chrono::milliseconds(1000));
    EXPECT_EQ(output1[1].monotonic_timestamp_time.boot, 0u);
    EXPECT_EQ(output1[1].monotonic_timestamp_time.time,
              e + chrono::milliseconds(2001));
    EXPECT_TRUE(output1[1].data != nullptr);

    EXPECT_EQ(output1[2].monotonic_event_time.boot, 1u);
    EXPECT_EQ(output1[2].monotonic_event_time.time,
              e + chrono::seconds(20) + chrono::milliseconds(2000));
    EXPECT_EQ(output1[2].monotonic_remote_time.boot, 0u);
    EXPECT_EQ(output1[2].monotonic_remote_time.time,
              e + chrono::milliseconds(2000));
    EXPECT_EQ(output1[2].remote_queue_index,
              (BootQueueIndex{.boot = 0u, .index = 1u}));
    EXPECT_EQ(output1[2].monotonic_timestamp_time.boot, 0u);
    EXPECT_EQ(output1[2].monotonic_timestamp_time.time,
              e + chrono::milliseconds(2001));
    EXPECT_TRUE(output1[2].data != nullptr);

    EXPECT_EQ(output1[3].monotonic_event_time.boot, 1u);
    EXPECT_EQ(output1[3].monotonic_event_time.time,
              e + chrono::seconds(20) + chrono::milliseconds(3000));
    EXPECT_EQ(output1[3].monotonic_remote_time.boot, 0u);
    EXPECT_EQ(output1[3].monotonic_remote_time.time,
              e + chrono::milliseconds(3000));
    EXPECT_EQ(output1[3].remote_queue_index,
              (BootQueueIndex{.boot = 0u, .index = 2u}));
    EXPECT_EQ(output1[3].monotonic_timestamp_time.boot, 0u);
    EXPECT_EQ(output1[3].monotonic_timestamp_time.time,
              e + chrono::milliseconds(3001));
    EXPECT_TRUE(output1[3].data != nullptr);

    LOG(INFO) << output1[0];
    LOG(INFO) << output1[1];
    LOG(INFO) << output1[2];
    LOG(INFO) << output1[3];
  }
}

TEST_F(RebootTimestampMapperTest, Node2Reboot) {
  const aos::monotonic_clock::time_point e = monotonic_clock::epoch();
  {
    TestDetachedBufferWriter writer0a(logfile0_);
    writer0a.QueueSpan(boot0a_.span());
    TestDetachedBufferWriter writer0b(logfile1_);
    writer0b.QueueSpan(boot0b_.span());
    TestDetachedBufferWriter writer1a(logfile2_);
    writer1a.QueueSpan(boot1a_.span());
    TestDetachedBufferWriter writer1b(logfile3_);
    writer1b.QueueSpan(boot1b_.span());

    writer1a.WriteSizedFlatbuffer(MakeLogMessage(
        e + chrono::seconds(100) + chrono::milliseconds(1000), 3, 0x005));
    writer0a.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::seconds(100) + chrono::milliseconds(1000), 3,
        chrono::seconds(-100),
        e + chrono::seconds(100) + chrono::milliseconds(1001)));

    writer1b.WriteSizedFlatbuffer(MakeLogMessage(
        e + chrono::seconds(20) + chrono::milliseconds(2000), 3, 0x006));
    writer0b.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::seconds(20) + chrono::milliseconds(2000), 3,
        chrono::seconds(-20),
        e + chrono::seconds(20) + chrono::milliseconds(2001)));

    writer1b.WriteSizedFlatbuffer(MakeLogMessage(
        e + chrono::seconds(20) + chrono::milliseconds(3000), 3, 0x007));
    writer0b.WriteSizedFlatbuffer(MakeTimestampMessage(
        e + chrono::seconds(20) + chrono::milliseconds(3000), 3,
        chrono::seconds(-20),
        e + chrono::seconds(20) + chrono::milliseconds(3001)));
  }

  const std::vector<LogFile> parts =
      SortParts({logfile0_, logfile1_, logfile2_, logfile3_});

  for (const auto &x : parts) {
    LOG(INFO) << x;
  }
  ASSERT_EQ(parts.size(), 1u);
  ASSERT_EQ(parts[0].logger_node, "pi1");

  size_t mapper0_count = 0;
  TimestampMapper mapper0(FilterPartsForNode(parts, "pi1"));
  mapper0.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper0_count; });
  size_t mapper1_count = 0;
  TimestampMapper mapper1(FilterPartsForNode(parts, "pi2"));
  mapper1.set_timestamp_callback(
      [&](TimestampedMessage *) { ++mapper1_count; });

  mapper0.AddPeer(&mapper1);
  mapper1.AddPeer(&mapper0);

  {
    std::deque<TimestampedMessage> output0;

    EXPECT_EQ(mapper0_count, 0u);
    EXPECT_EQ(mapper1_count, 0u);
    ASSERT_TRUE(mapper0.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 1u);
    EXPECT_EQ(mapper1_count, 0u);
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    EXPECT_TRUE(mapper0.started());
    EXPECT_EQ(mapper0_count, 1u);
    EXPECT_EQ(mapper1_count, 0u);

    ASSERT_TRUE(mapper0.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 2u);
    EXPECT_EQ(mapper1_count, 0u);
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    EXPECT_TRUE(mapper0.started());

    ASSERT_TRUE(mapper0.Front() != nullptr);
    output0.emplace_back(std::move(*mapper0.Front()));
    mapper0.PopFront();
    EXPECT_TRUE(mapper0.started());

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 0u);

    ASSERT_TRUE(mapper0.Front() == nullptr);

    LOG(INFO) << output0[0];
    LOG(INFO) << output0[1];
    LOG(INFO) << output0[2];

    EXPECT_EQ(output0[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[0].monotonic_event_time.time,
              e + chrono::milliseconds(1000));
    EXPECT_EQ(output0[0].monotonic_remote_time.boot, 0u);
    EXPECT_EQ(output0[0].monotonic_remote_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1000));
    EXPECT_EQ(output0[0].monotonic_timestamp_time.boot, 0u);
    EXPECT_EQ(output0[0].monotonic_timestamp_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1001));
    EXPECT_TRUE(output0[0].data != nullptr);

    EXPECT_EQ(output0[1].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[1].monotonic_event_time.time,
              e + chrono::milliseconds(2000));
    EXPECT_EQ(output0[1].monotonic_remote_time.boot, 1u);
    EXPECT_EQ(output0[1].monotonic_remote_time.time,
              e + chrono::seconds(20) + chrono::milliseconds(2000));
    EXPECT_EQ(output0[1].monotonic_timestamp_time.boot, 0u);
    EXPECT_EQ(output0[1].monotonic_timestamp_time.time,
              e + chrono::seconds(20) + chrono::milliseconds(2001));
    EXPECT_TRUE(output0[1].data != nullptr);

    EXPECT_EQ(output0[2].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output0[2].monotonic_event_time.time,
              e + chrono::milliseconds(3000));
    EXPECT_EQ(output0[2].monotonic_remote_time.boot, 1u);
    EXPECT_EQ(output0[2].monotonic_remote_time.time,
              e + chrono::seconds(20) + chrono::milliseconds(3000));
    EXPECT_EQ(output0[2].monotonic_timestamp_time.boot, 0u);
    EXPECT_EQ(output0[2].monotonic_timestamp_time.time,
              e + chrono::seconds(20) + chrono::milliseconds(3001));
    EXPECT_TRUE(output0[2].data != nullptr);
  }

  {
    SCOPED_TRACE("Trying node1 now");
    std::deque<TimestampedMessage> output1;

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 0u);

    ASSERT_TRUE(mapper1.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 1u);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 1u);

    ASSERT_TRUE(mapper1.Front() != nullptr);
    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 2u);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    ASSERT_TRUE(mapper1.Front() != nullptr);
    output1.emplace_back(std::move(*mapper1.Front()));
    mapper1.PopFront();
    EXPECT_TRUE(mapper1.started());

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 3u);

    ASSERT_TRUE(mapper1.Front() == nullptr);

    EXPECT_EQ(mapper0_count, 3u);
    EXPECT_EQ(mapper1_count, 3u);

    EXPECT_EQ(output1[0].monotonic_event_time.boot, 0u);
    EXPECT_EQ(output1[0].monotonic_event_time.time,
              e + chrono::seconds(100) + chrono::milliseconds(1000));
    EXPECT_EQ(output1[0].monotonic_remote_time, BootTimestamp::min_time());
    EXPECT_EQ(output1[0].monotonic_timestamp_time, BootTimestamp::min_time());
    EXPECT_TRUE(output1[0].data != nullptr);

    EXPECT_EQ(output1[1].monotonic_event_time.boot, 1u);
    EXPECT_EQ(output1[1].monotonic_event_time.time,
              e + chrono::seconds(20) + chrono::milliseconds(2000));
    EXPECT_EQ(output1[1].monotonic_remote_time, BootTimestamp::min_time());
    EXPECT_EQ(output1[1].monotonic_timestamp_time, BootTimestamp::min_time());
    EXPECT_TRUE(output1[1].data != nullptr);

    EXPECT_EQ(output1[2].monotonic_event_time.boot, 1u);
    EXPECT_EQ(output1[2].monotonic_event_time.time,
              e + chrono::seconds(20) + chrono::milliseconds(3000));
    EXPECT_EQ(output1[2].monotonic_remote_time, BootTimestamp::min_time());
    EXPECT_EQ(output1[2].monotonic_timestamp_time, BootTimestamp::min_time());
    EXPECT_TRUE(output1[2].data != nullptr);

    LOG(INFO) << output1[0];
    LOG(INFO) << output1[1];
    LOG(INFO) << output1[2];
  }
}

class SortingDeathTest : public SortingElementTest {
 public:
  SortingDeathTest()
      : SortingElementTest(),
        part0_(MakeHeader(config_, R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi1"
  },
  "logger_node": {
    "name": "pi1"
  },
  "monotonic_start_time": 1000000,
  "realtime_start_time": 1000000000000,
  "logger_monotonic_start_time": 1000000,
  "logger_realtime_start_time": 1000000000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "parts_uuid": "ee4f5a98-77d0-4e01-af2f-bbb29e098ede",
  "parts_index": 0,
  "logger_instance_uuid": "1c3142ad-10a5-408d-a760-b63b73d3b904",
  "logger_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "source_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "boot_uuids": [
    "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
    "6ba4f28d-21a2-4d7f-83f4-ee365cf86464",
    ""
  ],
  "oldest_remote_monotonic_timestamps": [
    9223372036854775807,
    9223372036854775807,
    9223372036854775807
  ],
  "oldest_local_monotonic_timestamps": [
    9223372036854775807,
    9223372036854775807,
    9223372036854775807
  ],
  "oldest_remote_unreliable_monotonic_timestamps": [
    9223372036854775807,
    0,
    9223372036854775807
  ],
  "oldest_local_unreliable_monotonic_timestamps": [
    9223372036854775807,
    0,
    9223372036854775807
  ]
})")),
        part1_(MakeHeader(config_, R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi1"
  },
  "logger_node": {
    "name": "pi1"
  },
  "monotonic_start_time": 1000000,
  "realtime_start_time": 1000000000000,
  "logger_monotonic_start_time": 1000000,
  "logger_realtime_start_time": 1000000000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "parts_uuid": "ee4f5a98-77d0-4e01-af2f-bbb29e098ede",
  "parts_index": 1,
  "logger_instance_uuid": "1c3142ad-10a5-408d-a760-b63b73d3b904",
  "logger_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "source_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "boot_uuids": [
    "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
    "b728d27a-9181-4eac-bfc1-5d09b80469d2",
    ""
  ],
  "oldest_remote_monotonic_timestamps": [
    9223372036854775807,
    9223372036854775807,
    9223372036854775807
  ],
  "oldest_local_monotonic_timestamps": [
    9223372036854775807,
    9223372036854775807,
    9223372036854775807
  ],
  "oldest_remote_unreliable_monotonic_timestamps": [
    9223372036854775807,
    100000,
    9223372036854775807
  ],
  "oldest_local_unreliable_monotonic_timestamps": [
    9223372036854775807,
    100000,
    9223372036854775807
  ]
})")),
        part2_(MakeHeader(config_, R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi1"
  },
  "logger_node": {
    "name": "pi1"
  },
  "monotonic_start_time": 1000000,
  "realtime_start_time": 1000000000000,
  "logger_monotonic_start_time": 1000000,
  "logger_realtime_start_time": 1000000000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "parts_uuid": "ee4f5a98-77d0-4e01-af2f-bbb29e098ede",
  "parts_index": 2,
  "logger_instance_uuid": "1c3142ad-10a5-408d-a760-b63b73d3b904",
  "logger_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "source_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "boot_uuids": [
    "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
    "6ba4f28d-21a2-4d7f-83f4-ee365cf86464",
    ""
  ],
  "oldest_remote_monotonic_timestamps": [
    9223372036854775807,
    9223372036854775807,
    9223372036854775807
  ],
  "oldest_local_monotonic_timestamps": [
    9223372036854775807,
    9223372036854775807,
    9223372036854775807
  ],
  "oldest_remote_unreliable_monotonic_timestamps": [
    9223372036854775807,
    200000,
    9223372036854775807
  ],
  "oldest_local_unreliable_monotonic_timestamps": [
    9223372036854775807,
    200000,
    9223372036854775807
  ]
})")),
        part3_(MakeHeader(config_, R"({
  /* 100ms */
  "max_out_of_order_duration": 100000000,
  "node": {
    "name": "pi1"
  },
  "logger_node": {
    "name": "pi1"
  },
  "monotonic_start_time": 1000000,
  "realtime_start_time": 1000000000000,
  "logger_monotonic_start_time": 1000000,
  "logger_realtime_start_time": 1000000000000,
  "log_event_uuid": "30ef1283-81d7-4004-8c36-1c162dbcb2b2",
  "parts_uuid": "ee4f5a98-77d0-4e01-af2f-bbb29e098ede",
  "parts_index": 3,
  "logger_instance_uuid": "1c3142ad-10a5-408d-a760-b63b73d3b904",
  "logger_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "source_node_boot_uuid": "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
  "boot_uuids": [
    "a570df8b-5cc2-4dbe-89bd-286f9ddd02b7",
    "b728d27a-9181-4eac-bfc1-5d09b80469d2",
    ""
  ],
  "oldest_remote_monotonic_timestamps": [
    9223372036854775807,
    9223372036854775807,
    9223372036854775807
  ],
  "oldest_local_monotonic_timestamps": [
    9223372036854775807,
    9223372036854775807,
    9223372036854775807
  ],
  "oldest_remote_unreliable_monotonic_timestamps": [
    9223372036854775807,
    300000,
    9223372036854775807
  ],
  "oldest_local_unreliable_monotonic_timestamps": [
    9223372036854775807,
    300000,
    9223372036854775807
  ]
})")) {}

 protected:
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> part0_;
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> part1_;
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> part2_;
  const aos::SizePrefixedFlatbufferDetachedBuffer<LogFileHeader> part3_;
};

// Tests that if 2 computers go back and forth trying to be the same node, we
// die in sorting instead of failing to estimate time.
TEST_F(SortingDeathTest, FightingNodes) {
  {
    TestDetachedBufferWriter writer0(logfile0_);
    writer0.QueueSpan(part0_.span());
    TestDetachedBufferWriter writer1(logfile1_);
    writer1.QueueSpan(part1_.span());
    TestDetachedBufferWriter writer2(logfile2_);
    writer2.QueueSpan(part2_.span());
    TestDetachedBufferWriter writer3(logfile3_);
    writer3.QueueSpan(part3_.span());
  }

  EXPECT_DEATH(
      {
        const std::vector<LogFile> parts =
            SortParts({logfile0_, logfile1_, logfile2_, logfile3_});
      },
      "found overlapping boots on");
}

// Tests that we MessageReader blows up on a bad message.
TEST(MessageReaderConfirmCrash, ReadWrite) {
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
  const aos::SizePrefixedFlatbufferDetachedBuffer<MessageHeader> m4 =
      JsonToSizedFlatbuffer<MessageHeader>(
          R"({ "channel_index": 0, "monotonic_sent_time": 4 })");

  // Starts out like a proper flat buffer header, but it breaks down ...
  std::vector<uint8_t> garbage{8, 0, 0, 0, 16, 0, 0, 0, 4, 0, 0, 0};
  absl::Span<uint8_t> m3_span(garbage);

  {
    TestDetachedBufferWriter writer(logfile);
    writer.QueueSpan(config.span());
    writer.QueueSpan(m1.span());
    writer.QueueSpan(m2.span());
    writer.QueueSpan(m3_span);
    writer.QueueSpan(m4.span());  // This message is "hidden"
  }

  {
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
    // Confirm default crashing behavior
    EXPECT_DEATH(reader.ReadMessage(), "Corrupted message at offset");
  }

  {
    gflags::FlagSaver fs;

    MessageReader reader(logfile);
    reader.set_crash_on_corrupt_message_flag(false);

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
    // Confirm avoiding the corrupted message crash, stopping instead.
    EXPECT_FALSE(reader.ReadMessage());
  }

  {
    gflags::FlagSaver fs;

    MessageReader reader(logfile);
    reader.set_crash_on_corrupt_message_flag(false);
    reader.set_ignore_corrupt_messages_flag(true);

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
    // Confirm skipping of the corrupted message to read the hidden one.
    EXPECT_TRUE(reader.ReadMessage());
    EXPECT_EQ(reader.newest_timestamp(),
              monotonic_clock::time_point(chrono::nanoseconds(4)));
    EXPECT_FALSE(reader.ReadMessage());
  }
}

class InlinePackMessage : public ::testing::Test {
 protected:
  aos::Context RandomContext() {
    data_ = RandomData();
    std::uniform_int_distribution<uint32_t> uint32_distribution(
        std::numeric_limits<uint32_t>::min(),
        std::numeric_limits<uint32_t>::max());

    std::uniform_int_distribution<int64_t> time_distribution(
        std::numeric_limits<int64_t>::min(),
        std::numeric_limits<int64_t>::max());

    aos::Context context;
    context.monotonic_event_time =
        aos::monotonic_clock::epoch() +
        chrono::nanoseconds(time_distribution(random_number_generator_));
    context.realtime_event_time =
        aos::realtime_clock::epoch() +
        chrono::nanoseconds(time_distribution(random_number_generator_));

    context.monotonic_remote_time =
        aos::monotonic_clock::epoch() +
        chrono::nanoseconds(time_distribution(random_number_generator_));
    context.realtime_remote_time =
        aos::realtime_clock::epoch() +
        chrono::nanoseconds(time_distribution(random_number_generator_));

    context.queue_index = uint32_distribution(random_number_generator_);
    context.remote_queue_index = uint32_distribution(random_number_generator_);
    context.size = data_.size();
    context.data = data_.data();
    return context;
  }

  aos::monotonic_clock::time_point RandomMonotonic() {
    std::uniform_int_distribution<int64_t> time_distribution(
        0, std::numeric_limits<int64_t>::max());
    return aos::monotonic_clock::epoch() +
           chrono::nanoseconds(time_distribution(random_number_generator_));
  }

  aos::SizePrefixedFlatbufferDetachedBuffer<message_bridge::RemoteMessage>
  RandomRemoteMessage() {
    std::uniform_int_distribution<uint8_t> uint8_distribution(
        std::numeric_limits<uint8_t>::min(),
        std::numeric_limits<uint8_t>::max());

    std::uniform_int_distribution<int64_t> time_distribution(
        std::numeric_limits<int64_t>::min(),
        std::numeric_limits<int64_t>::max());

    flatbuffers::FlatBufferBuilder fbb;
    message_bridge::RemoteMessage::Builder builder(fbb);
    builder.add_queue_index(uint8_distribution(random_number_generator_));

    builder.add_monotonic_sent_time(
        time_distribution(random_number_generator_));
    builder.add_realtime_sent_time(time_distribution(random_number_generator_));
    builder.add_monotonic_remote_time(
        time_distribution(random_number_generator_));
    builder.add_realtime_remote_time(
        time_distribution(random_number_generator_));

    builder.add_remote_queue_index(
        uint8_distribution(random_number_generator_));

    fbb.FinishSizePrefixed(builder.Finish());
    return fbb.Release();
  }

  std::vector<uint8_t> RandomData() {
    std::vector<uint8_t> result;
    std::uniform_int_distribution<int> length_distribution(1, 32);
    std::uniform_int_distribution<uint8_t> data_distribution(
        std::numeric_limits<uint8_t>::min(),
        std::numeric_limits<uint8_t>::max());

    const size_t length = length_distribution(random_number_generator_);

    result.reserve(length);
    for (size_t i = 0; i < length; ++i) {
      result.emplace_back(data_distribution(random_number_generator_));
    }
    return result;
  }

  std::mt19937 random_number_generator_{
      std::mt19937(::aos::testing::RandomSeed())};

  std::vector<uint8_t> data_;
};

// Uses the binary schema to annotate a provided flatbuffer.  Returns the
// annotated flatbuffer.
std::string AnnotateBinaries(
    const aos::NonSizePrefixedFlatbuffer<reflection::Schema> &schema,
    const std::string &schema_filename,
    flatbuffers::span<uint8_t> binary_data) {
  flatbuffers::BinaryAnnotator binary_annotator(
      schema.span().data(), schema.span().size(), binary_data.data(),
      binary_data.size());

  auto annotations = binary_annotator.Annotate();

  flatbuffers::AnnotatedBinaryTextGenerator text_generator(
      flatbuffers::AnnotatedBinaryTextGenerator::Options{}, annotations,
      binary_data.data(), binary_data.size());

  text_generator.Generate(aos::testing::TestTmpDir() + "/foo.bfbs",
                          schema_filename);

  return aos::util::ReadFileToStringOrDie(aos::testing::TestTmpDir() +
                                          "/foo.afb");
}

// Event loop which just has working time functions for the Copier classes
// tested below.
class TimeEventLoop : public EventLoop {
 public:
  TimeEventLoop() : EventLoop(nullptr) {}

  aos::monotonic_clock::time_point monotonic_now() const final {
    return aos::monotonic_clock::min_time;
  }
  realtime_clock::time_point realtime_now() const final {
    return aos::realtime_clock::min_time;
  }

  void OnRun(::std::function<void()> /*on_run*/) final { LOG(FATAL); }

  const std::string_view name() const final { return "time"; }
  const Node *node() const final { return nullptr; }

  void SetRuntimeAffinity(const cpu_set_t & /*cpuset*/) final { LOG(FATAL); }
  void SetRuntimeRealtimePriority(int /*priority*/) final { LOG(FATAL); }

  const cpu_set_t &runtime_affinity() const final {
    LOG(FATAL);
    return cpuset_;
  }

  TimerHandler *AddTimer(::std::function<void()> /*callback*/) final {
    LOG(FATAL);
    return nullptr;
  }

  std::unique_ptr<RawSender> MakeRawSender(const Channel * /*channel*/) final {
    LOG(FATAL);
    return std::unique_ptr<RawSender>();
  }

  const UUID &boot_uuid() const final {
    LOG(FATAL);
    return boot_uuid_;
  }

  void set_name(const std::string_view name) final { LOG(FATAL) << name; }

  pid_t GetTid() final {
    LOG(FATAL);
    return 0;
  }

  int NumberBuffers(const Channel * /*channel*/) final {
    LOG(FATAL);
    return 0;
  }

  int runtime_realtime_priority() const final {
    LOG(FATAL);
    return 0;
  }

  std::unique_ptr<RawFetcher> MakeRawFetcher(
      const Channel * /*channel*/) final {
    LOG(FATAL);
    return std::unique_ptr<RawFetcher>();
  }

  PhasedLoopHandler *AddPhasedLoop(
      ::std::function<void(int)> /*callback*/,
      const monotonic_clock::duration /*interval*/,
      const monotonic_clock::duration /*offset*/) final {
    LOG(FATAL);
    return nullptr;
  }

  void MakeRawWatcher(
      const Channel * /*channel*/,
      std::function<void(const Context &context, const void *message)>
      /*watcher*/) final {
    LOG(FATAL);
  }

 private:
  const cpu_set_t cpuset_ = DefaultAffinity();
  UUID boot_uuid_ = UUID ::Zero();
};

// Tests that all variations of PackMessage are equivalent to the inline
// PackMessage used to avoid allocations.
TEST_F(InlinePackMessage, Equivilent) {
  std::uniform_int_distribution<uint32_t> uint32_distribution(
      std::numeric_limits<uint32_t>::min(),
      std::numeric_limits<uint32_t>::max());
  aos::FlatbufferVector<reflection::Schema> schema =
      FileToFlatbuffer<reflection::Schema>(
          ArtifactPath("aos/events/logging/logger.bfbs"));

  for (const LogType type :
       {LogType::kLogMessage, LogType::kLogDeliveryTimeOnly,
        LogType::kLogMessageAndDeliveryTime, LogType::kLogRemoteMessage}) {
    for (int i = 0; i < 100; ++i) {
      aos::Context context = RandomContext();
      const uint32_t channel_index =
          uint32_distribution(random_number_generator_);

      flatbuffers::FlatBufferBuilder fbb;
      fbb.ForceDefaults(true);
      fbb.FinishSizePrefixed(PackMessage(&fbb, context, channel_index, type));

      VLOG(1) << absl::BytesToHexString(std::string_view(
          reinterpret_cast<const char *>(fbb.GetBufferSpan().data()),
          fbb.GetBufferSpan().size()));

      // Make sure that both the builder and inline method agree on sizes.
      ASSERT_EQ(fbb.GetSize(), PackMessageSize(type, context.size))
          << "log type " << static_cast<int>(type);

      // Initialize the buffer to something nonzero to make sure all the padding
      // bytes are set to 0.
      std::vector<uint8_t> repacked_message(PackMessageSize(type, context.size),
                                            67);

      // And verify packing inline works as expected.
      EXPECT_EQ(
          repacked_message.size(),
          PackMessageInline(repacked_message.data(), context, channel_index,
                            type, 0u, repacked_message.size()));
      EXPECT_EQ(absl::Span<uint8_t>(repacked_message),
                absl::Span<uint8_t>(fbb.GetBufferSpan().data(),
                                    fbb.GetBufferSpan().size()))
          << AnnotateBinaries(schema, "aos/events/logging/logger.bfbs",
                              fbb.GetBufferSpan());

      // Ok, now we want to confirm that we can build up arbitrary pieces of
      // said flatbuffer.  Try all of them since it is cheap.
      TimeEventLoop event_loop;
      for (size_t i = 0; i < repacked_message.size(); i += 8) {
        for (size_t j = i; j < repacked_message.size(); j += 8) {
          std::vector<uint8_t> destination(repacked_message.size(), 67u);
          ContextDataCopier copier(context, channel_index, type, &event_loop);

          copier.Copy(destination.data(), i, j);

          size_t index = 0;
          for (size_t k = i; k < j; ++k) {
            ASSERT_EQ(destination[index], repacked_message[k])
                << ": Failed to match type " << static_cast<int>(type)
                << ", index " << index << " while testing range " << i << " to "
                << j;
            ;
            ++index;
          }
          // Now, confirm that none of the other bytes have been touched.
          for (; index < destination.size(); ++index) {
            ASSERT_EQ(destination[index], 67u);
          }
        }
      }
    }
  }
}

// Tests that all variations of PackMessage are equivilent to the inline
// PackMessage used to avoid allocations.
TEST_F(InlinePackMessage, RemoteEquivilent) {
  aos::FlatbufferVector<reflection::Schema> schema =
      FileToFlatbuffer<reflection::Schema>(
          ArtifactPath("aos/events/logging/logger.bfbs"));
  std::uniform_int_distribution<uint8_t> uint8_distribution(
      std::numeric_limits<uint8_t>::min(), std::numeric_limits<uint8_t>::max());

  for (int i = 0; i < 100; ++i) {
    aos::SizePrefixedFlatbufferDetachedBuffer<RemoteMessage> random_msg =
        RandomRemoteMessage();
    const size_t channel_index = uint8_distribution(random_number_generator_);
    const monotonic_clock::time_point monotonic_timestamp_time =
        RandomMonotonic();

    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(true);
    fbb.FinishSizePrefixed(PackRemoteMessage(
        &fbb, &random_msg.message(), channel_index, monotonic_timestamp_time));

    VLOG(1) << absl::BytesToHexString(std::string_view(
        reinterpret_cast<const char *>(fbb.GetBufferSpan().data()),
        fbb.GetBufferSpan().size()));

    // Make sure that both the builder and inline method agree on sizes.
    ASSERT_EQ(fbb.GetSize(), PackRemoteMessageSize());

    // Initialize the buffer to something nonzer to make sure all the padding
    // bytes are set to 0.
    std::vector<uint8_t> repacked_message(PackRemoteMessageSize(), 67);

    // And verify packing inline works as expected.
    EXPECT_EQ(repacked_message.size(),
              PackRemoteMessageInline(
                  repacked_message.data(), &random_msg.message(), channel_index,
                  monotonic_timestamp_time, 0u, repacked_message.size()));
    EXPECT_EQ(absl::Span<uint8_t>(repacked_message),
              absl::Span<uint8_t>(fbb.GetBufferSpan().data(),
                                  fbb.GetBufferSpan().size()))
        << AnnotateBinaries(schema, "aos/events/logging/logger.bfbs",
                            fbb.GetBufferSpan());

    // Ok, now we want to confirm that we can build up arbitrary pieces of said
    // flatbuffer.  Try all of them since it is cheap.
    TimeEventLoop event_loop;
    for (size_t i = 0; i < repacked_message.size(); i += 8) {
      for (size_t j = i; j < repacked_message.size(); j += 8) {
        std::vector<uint8_t> destination(repacked_message.size(), 67u);
        RemoteMessageCopier copier(&random_msg.message(), channel_index,
                                   monotonic_timestamp_time, &event_loop);

        copier.Copy(destination.data(), i, j);

        size_t index = 0;
        for (size_t k = i; k < j; ++k) {
          ASSERT_EQ(destination[index], repacked_message[k]);
          ++index;
        }
        for (; index < destination.size(); ++index) {
          ASSERT_EQ(destination[index], 67u);
        }
      }
    }
  }
}

}  // namespace testing
}  // namespace logger
}  // namespace aos
