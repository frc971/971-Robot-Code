#include "aos/events/logging/logfile_utils.h"

#include "gtest/gtest.h"

#include "aos/events/logging/test_message_generated.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/tmpdir.h"

namespace aos {
namespace logger {
namespace testing {

template <typename T>
SizePrefixedFlatbufferDetachedBuffer<T> JsonToSizedFlatbuffer(
    const std::string_view data) {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  fbb.FinishSizePrefixed(JsonToFlatbuffer<T>(data, &fbb));
  return fbb.Release();
}

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

}  // namespace testing
}  // namespace logger
}  // namespace aos
