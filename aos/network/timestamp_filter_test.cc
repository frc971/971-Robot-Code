#include "aos/network/timestamp_filter.h"

#include <chrono>

#include "aos/macros.h"
#include "gtest/gtest.h"

namespace aos {
namespace message_bridge {
namespace testing {

namespace chrono = std::chrono;
using aos::monotonic_clock;

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

// Tests that the FromInt64 function correctly produces a mpq even though it
// only can use 32 bit numbers.
TEST(LineTest, Int64) {
  EXPECT_EQ(FromInt64(0x9710000000ll),
            mpq_class(0x971) * mpq_class(0x10000000));

  EXPECT_EQ(FromInt64(-0x9710000000ll),
            mpq_class(-0x971) * mpq_class(0x10000000));
}

// Tests that we can create a simple line and the methods return sane results.
TEST(LineTest, SimpleLine) {
  mpq_class offset(1023);
  mpq_class slope(1);
  Line l(offset, slope);

  EXPECT_EQ(l.mpq_offset(), offset);
  EXPECT_EQ(l.mpq_slope(), slope);

  EXPECT_EQ(l.offset(), chrono::nanoseconds(1023));
  EXPECT_EQ(l.slope(), 1.0);

  EXPECT_EQ(chrono::nanoseconds(1023 + 100),
            l.Eval(monotonic_clock::time_point(chrono::nanoseconds(100))));
}

// Tests that we can fit a line to 2 points and they recover correctly.
TEST(LineTest, FitLine) {
  const monotonic_clock::time_point ta(chrono::nanoseconds(1000));
  const monotonic_clock::time_point tb(chrono::nanoseconds(100001013));
  Line l = Line::Fit(std::make_tuple(ta, chrono::nanoseconds(100)),
                     std::make_tuple(tb, chrono::nanoseconds(105)));

  EXPECT_EQ(chrono::nanoseconds(100), l.Eval(ta));
  EXPECT_EQ(chrono::nanoseconds(105), l.Eval(tb));
}

// Tests that averaging 2 lines results in the correct outcome.
// Try to compute the correct outcome a couple of different ways to confirm the
// math is done right.
TEST(LineTest, AverageFits) {
  const monotonic_clock::time_point ta(chrono::nanoseconds(1000));
  const monotonic_clock::time_point tb(chrono::nanoseconds(100001013));

  // Test 2 lines which both diverge and should average back to nothing.
  {
    Line l1(mpq_class(999000), mpq_class(1000, 1000));
    Line l2(-mpq_class(990000), mpq_class(1000, 1000));

    Line a = AverageFits(l1, l2);
    a.Debug();

    EXPECT_EQ(a.mpq_slope(), mpq_class(0));

    // Confirm some points to make sure everything works.
    //
    // tb = ta + O(ta)
    // tb = Oa(ta) + ta
    // ta = Ob(tb) + tb
    // tb - ta = O(ta, tb)
    // So, if we pick a point at t=x, we can evaluate both functions and should
    // get back O(x)

    const monotonic_clock::time_point ta(chrono::nanoseconds(1000));
    const monotonic_clock::time_point tb(chrono::nanoseconds(100001013));

    EXPECT_EQ((l1.Eval(ta) - l2.Eval(a.Eval(ta) + ta)) / 2, a.Eval(ta));
    EXPECT_EQ((l1.Eval(tb) - l2.Eval(a.Eval(tb) + tb)) / 2, a.Eval(tb));
  }

  // Test 2 lines which are parallel, so there should be a slope.
  {
    Line l1(mpq_class(990000), mpq_class(1000, 1000));
    Line l2(-mpq_class(990000), -mpq_class(1000, 1000));

    Line a = AverageFits(l1, l2);
    a.Debug();

    EXPECT_EQ(a.mpq_slope(), mpq_class(2));

    // Confirm some points to make sure everything works.

    EXPECT_EQ((l1.Eval(ta) - l2.Eval(a.Eval(ta) + ta)) / 2, a.Eval(ta));
    EXPECT_EQ((l1.Eval(tb) - l2.Eval(a.Eval(tb) + tb)) / 2, a.Eval(tb));
  }
}

}  // namespace testing
}  // namespace message_bridge
}  // namespace aos
