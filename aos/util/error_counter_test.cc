#include "aos/util/error_counter.h"

#include "gtest/gtest.h"

#include "aos/events/event_loop_generated.h"
#include "aos/flatbuffers.h"

namespace aos::util::testing {
// Exercises the basic API for the ErrorCounter class, ensuring that everything
// works in the normal case.
TEST(ErrorCounterTest, ErrorCounter) {
  ErrorCounter<aos::timing::SendError, aos::timing::SendErrorCount> counter;
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  const flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<aos::timing::SendErrorCount>>>
      counts_offset = counter.Initialize(&fbb);
  aos::timing::Sender::Builder builder(fbb);
  builder.add_error_counts(counts_offset);
  fbb.Finish(builder.Finish());
  aos::FlatbufferDetachedBuffer<aos::timing::Sender> message = fbb.Release();
  counter.set_mutable_vector(message.mutable_message()->mutable_error_counts());
  counter.IncrementError(aos::timing::SendError::MESSAGE_SENT_TOO_FAST);
  counter.IncrementError(aos::timing::SendError::MESSAGE_SENT_TOO_FAST);
  counter.IncrementError(aos::timing::SendError::INVALID_REDZONE);
  ASSERT_EQ(2u, message.message().error_counts()->size());
  EXPECT_EQ(aos::timing::SendError::MESSAGE_SENT_TOO_FAST,
            message.message().error_counts()->Get(0)->error());
  EXPECT_EQ(2u, message.message().error_counts()->Get(0)->count());
  EXPECT_EQ(aos::timing::SendError::INVALID_REDZONE,
            message.message().error_counts()->Get(1)->error());
  EXPECT_EQ(1u, message.message().error_counts()->Get(1)->count());

  counter.ResetCounts();
  EXPECT_EQ(0u, message.message().error_counts()->Get(0)->count());
  EXPECT_EQ(0u, message.message().error_counts()->Get(1)->count());
}

// Tests the ArrayErrorCounter
TEST(ErrorCounterTest, ARrayErrorCounter) {
  ArrayErrorCounter<aos::timing::SendError, aos::timing::SendErrorCount>
      counter;
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  counter.IncrementError(aos::timing::SendError::MESSAGE_SENT_TOO_FAST);
  counter.IncrementError(aos::timing::SendError::MESSAGE_SENT_TOO_FAST);
  counter.IncrementError(aos::timing::SendError::INVALID_REDZONE);
  {
    const flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<aos::timing::SendErrorCount>>>
        counts_offset = counter.PopulateCounts(&fbb);
    aos::timing::Sender::Builder builder(fbb);
    builder.add_error_counts(counts_offset);
    fbb.Finish(builder.Finish());
    aos::FlatbufferDetachedBuffer<aos::timing::Sender> message = fbb.Release();
    ASSERT_EQ(2u, message.message().error_counts()->size());
    EXPECT_EQ(aos::timing::SendError::MESSAGE_SENT_TOO_FAST,
              message.message().error_counts()->Get(0)->error());
    EXPECT_EQ(2u, message.message().error_counts()->Get(0)->count());
    EXPECT_EQ(aos::timing::SendError::INVALID_REDZONE,
              message.message().error_counts()->Get(1)->error());
    EXPECT_EQ(1u, message.message().error_counts()->Get(1)->count());
  }

  counter.ResetCounts();
  {
    const flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<aos::timing::SendErrorCount>>>
        counts_offset = counter.PopulateCounts(&fbb);
    aos::timing::Sender::Builder builder(fbb);
    builder.add_error_counts(counts_offset);
    fbb.Finish(builder.Finish());
    aos::FlatbufferDetachedBuffer<aos::timing::Sender> message = fbb.Release();
    ASSERT_EQ(2u, message.message().error_counts()->size());
    EXPECT_EQ(0u, message.message().error_counts()->Get(0)->count());
    EXPECT_EQ(0u, message.message().error_counts()->Get(1)->count());
  }
}
}  // namespace aos::util::testing
