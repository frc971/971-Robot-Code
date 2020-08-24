#include "aos/network/timestamp_filter.h"

#include <chrono>

#include "aos/macros.h"
#include "gtest/gtest.h"

namespace aos {
namespace message_bridge {
namespace testing {

namespace chrono = std::chrono;

// Tests that adding samples tracks more negative offsets down quickly, and
// slowly comes back up.
TEST(TimestampFilterTest, Sample) {
  TimestampFilter filter;

  EXPECT_EQ(filter.offset(), 0.0);
  EXPECT_EQ(filter.base_offset(), chrono::seconds(0));
  EXPECT_FALSE(filter.has_sample());

  filter.Sample(aos::monotonic_clock::epoch() + chrono::seconds(1),
                chrono::milliseconds(-100));

  EXPECT_EQ(filter.offset(), -0.1);
  EXPECT_EQ(filter.base_offset(), chrono::seconds(0));
  EXPECT_TRUE(filter.has_sample());

  // Further negative -> follow the min down exactly.
  filter.Sample(aos::monotonic_clock::epoch() + chrono::seconds(2),
                chrono::milliseconds(-1000));

  EXPECT_EQ(filter.offset(), -1.0);
  EXPECT_EQ(filter.base_offset(), chrono::seconds(0));
  EXPECT_TRUE(filter.has_sample());

  // Positive now goes positive, but slower.
  filter.Sample(aos::monotonic_clock::epoch() + chrono::seconds(3),
                chrono::milliseconds(0));

  // We have velocity now, so we will continue.
  EXPECT_GT(filter.offset(), -1.001);
  EXPECT_LT(filter.offset(), -1.0001);
  EXPECT_EQ(filter.base_offset(), chrono::seconds(0));
  EXPECT_TRUE(filter.has_sample());
}

// Tests that ClippedAverageFilter tracks between the two filters.
TEST(ClippedAverageFilterTest, Sample) {
  ClippedAverageFilter filter;

  // Pass in a sample in both the forward and reverse direction.  We should
  // expect that the offset should be smack down the middle.
  filter.FwdSample(aos::monotonic_clock::epoch() + chrono::seconds(1),
                   chrono::milliseconds(101));

  filter.RevSample(aos::monotonic_clock::epoch() + chrono::seconds(1),
                   chrono::milliseconds(-100));

  EXPECT_EQ(filter.offset(), chrono::microseconds(100500));

  // Confirm the base offset works too.
  filter.set_base_offset(chrono::milliseconds(100));

  filter.FwdSample(aos::monotonic_clock::epoch() + chrono::seconds(1),
                   chrono::milliseconds(101));

  filter.RevSample(aos::monotonic_clock::epoch() + chrono::seconds(1),
                   chrono::milliseconds(-100));

  EXPECT_EQ(filter.offset(), chrono::microseconds(100500));
}

}  // namespace testing
}  // namespace message_bridge
}  // namespace aos
