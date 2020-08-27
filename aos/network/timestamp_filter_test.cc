#include "aos/network/timestamp_filter.h"

#include <chrono>

#include "aos/configuration.h"
#include "aos/json_to_flatbuffer.h"
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

// Tests that 2 samples results in the correct line between them, and the
// correct intermediate as it is being built.
TEST(NoncausalTimestampFilterTest, SingleSample) {
  const monotonic_clock::time_point ta(chrono::nanoseconds(100000));
  const monotonic_clock::time_point tb(chrono::nanoseconds(200000));

  NoncausalTimestampFilter filter;

  filter.Sample(ta, chrono::nanoseconds(1000));
  EXPECT_EQ(filter.timestamps().size(), 1u);

  {
    Line l1 = filter.FitLine();

    EXPECT_EQ(l1.mpq_offset(), mpq_class(1000));
    EXPECT_EQ(l1.mpq_slope(), mpq_class(0));
  }

  filter.Sample(tb, chrono::nanoseconds(1100));
  EXPECT_EQ(filter.timestamps().size(), 2u);

  {
    Line l2 = filter.FitLine();
    EXPECT_EQ(l2.mpq_offset(), mpq_class(900));
    EXPECT_EQ(l2.mpq_slope(), mpq_class(1, 1000));
  }
}

// Tests that invalid samples get clipped as expected.
TEST(NoncausalTimestampFilterTest, ClippedSample) {
  const monotonic_clock::time_point ta(chrono::milliseconds(0));
  const monotonic_clock::time_point tb(chrono::milliseconds(1));
  const monotonic_clock::time_point tc(chrono::milliseconds(2));

  {
    // A positive slope of 1 ms/second is properly applied.
    NoncausalTimestampFilter filter;

    filter.Sample(ta, chrono::microseconds(1));
    filter.Debug();
    filter.Sample(tb, chrono::microseconds(2));
    filter.Debug();
    ASSERT_EQ(filter.timestamps().size(), 2u);

    {
      Line l2 = filter.FitLine();
      EXPECT_EQ(l2.mpq_offset(), mpq_class(1000));
      EXPECT_EQ(l2.mpq_slope(), mpq_class(1, 1000));
    }
  }

  {
    // A negative slope of 1 ms/second is properly applied.
    NoncausalTimestampFilter filter;

    filter.Sample(ta, chrono::microseconds(1));
    filter.Debug();
    filter.Sample(tb, chrono::microseconds(0));
    filter.Debug();
    ASSERT_EQ(filter.timestamps().size(), 2u);

    {
      Line l2 = filter.FitLine();
      EXPECT_EQ(l2.mpq_offset(), mpq_class(1000));
      EXPECT_EQ(l2.mpq_slope(), -mpq_class(1, 1000));
    }
  }

  {
    // Too much negative is ignored.
    NoncausalTimestampFilter filter;

    filter.Sample(ta, chrono::microseconds(1));
    filter.Debug();
    filter.Sample(tb, -chrono::microseconds(1));
    filter.Debug();
    ASSERT_EQ(filter.timestamps().size(), 1u);
  }

  {
    // Too much positive pulls up the first point.
    NoncausalTimestampFilter filter;

    filter.Sample(ta, chrono::microseconds(1));
    filter.Debug();
    filter.Sample(tb, chrono::microseconds(3));
    filter.Debug();
    ASSERT_EQ(filter.timestamps().size(), 2u);

    EXPECT_EQ(std::get<1>(filter.timestamps()[0]), chrono::microseconds(2));
    EXPECT_EQ(std::get<1>(filter.timestamps()[1]), chrono::microseconds(3));
  }

  {
    // Too much positive slope removes points.
    NoncausalTimestampFilter filter;

    filter.Sample(ta, chrono::microseconds(1));
    filter.Debug();
    filter.Sample(tb, chrono::microseconds(1));
    filter.Debug();
    ASSERT_EQ(filter.timestamps().size(), 2u);

    // Now add a sample with a slope of 0.002.  This should back propagate and
    // remove the middle point since it violates our constraints.
    filter.Sample(tc, chrono::microseconds(3));
    filter.Debug();
    ASSERT_EQ(filter.timestamps().size(), 2u);

    EXPECT_EQ(std::get<1>(filter.timestamps()[0]), chrono::microseconds(1));
    EXPECT_EQ(std::get<1>(filter.timestamps()[1]), chrono::microseconds(3));
  }
}

// Tests that removing points from the filter works as expected.
TEST(NoncausalTimestampFilterTest, PointRemoval) {
  const monotonic_clock::time_point t_before(-chrono::milliseconds(1));
  const monotonic_clock::time_point ta(chrono::milliseconds(0));
  const monotonic_clock::time_point tb(chrono::milliseconds(1));
  const monotonic_clock::time_point tc(chrono::milliseconds(2));

  // A positive slope of 1 ms/second is properly applied.
  NoncausalTimestampFilter filter;

  filter.Sample(ta, chrono::microseconds(1));
  filter.Debug();
  filter.Sample(tb, chrono::microseconds(2));
  filter.Debug();
  filter.Sample(tc, chrono::microseconds(1));
  filter.Debug();
  ASSERT_EQ(filter.timestamps().size(), 3u);

  // Before or in the middle of the first line segment shouldn't change the
  // number of points.
  EXPECT_FALSE(filter.Pop(t_before));
  ASSERT_EQ(filter.timestamps().size(), 3u);

  EXPECT_FALSE(filter.Pop(ta));
  ASSERT_EQ(filter.timestamps().size(), 3u);

  EXPECT_FALSE(filter.Pop(ta + chrono::microseconds(100)));
  ASSERT_EQ(filter.timestamps().size(), 3u);

  // The second point should trigger a pop, since the offset computed using the
  // points won't change when it is used, and any times after (even 1-2 ns
  // later) would be wrong.
  EXPECT_TRUE(filter.Pop(tb));
  ASSERT_EQ(filter.timestamps().size(), 2u);
}

// Run a couple of points through the estimator and confirm it works.
TEST(NoncausalOffsetEstimatorTest, FullEstimator) {
  const aos::FlatbufferDetachedBuffer<Node> node_a_buffer =
      JsonToFlatbuffer<Node>("{\"name\": \"a\"}");
  const aos::FlatbufferDetachedBuffer<Node> node_b_buffer =
      JsonToFlatbuffer<Node>("{\"name\": \"b\"}");

  const Node *node_a = &node_a_buffer.message();
  const Node *node_b = &node_b_buffer.message();

  const monotonic_clock::time_point ta1(chrono::milliseconds(1000));
  const monotonic_clock::time_point ta2 = ta1 + chrono::milliseconds(10);
  const monotonic_clock::time_point ta3 = ta1 + chrono::milliseconds(20);

  const monotonic_clock::time_point tb1(chrono::milliseconds(4000));
  const monotonic_clock::time_point tb2 =
      tb1 + chrono::milliseconds(10) + chrono::nanoseconds(100);
  const monotonic_clock::time_point tb3 = tb1 + chrono::milliseconds(20);

  NoncausalOffsetEstimator estimator(node_a, node_b);

  // Add 3 timestamps in and confirm that the slopes come out reasonably.
  estimator.Sample(node_a, ta1, tb1);
  estimator.Sample(node_b, tb1, ta1);
  EXPECT_EQ(estimator.a_timestamps().size(), 1u);
  EXPECT_EQ(estimator.b_timestamps().size(), 1u);

  // 1 point -> a line.
  EXPECT_EQ(estimator.fit().mpq_slope(), mpq_class(0));

  estimator.Sample(node_a, ta2, tb2);
  estimator.Sample(node_b, tb2, ta2);
  EXPECT_EQ(estimator.a_timestamps().size(), 2u);
  EXPECT_EQ(estimator.b_timestamps().size(), 2u);

  // Adding the second point should slope up.
  EXPECT_EQ(estimator.fit().mpq_slope(), mpq_class(1, 100000));

  estimator.Sample(node_a, ta3, tb3);
  estimator.Sample(node_b, tb3, ta3);
  EXPECT_EQ(estimator.a_timestamps().size(), 3u);
  EXPECT_EQ(estimator.b_timestamps().size(), 3u);

  // And the third point shouldn't change anything.
  EXPECT_EQ(estimator.fit().mpq_slope(), mpq_class(1, 100000));

  estimator.Pop(node_a, ta2);
  estimator.Pop(node_b, tb2);
  EXPECT_EQ(estimator.a_timestamps().size(), 2u);
  EXPECT_EQ(estimator.b_timestamps().size(), 2u);

  // Dropping the first point should have the slope point back down.
  EXPECT_EQ(estimator.fit().mpq_slope(), mpq_class(-1, 100000));

  // And dropping down to 1 point means 0 slope.
  estimator.Pop(node_a, ta3);
  estimator.Pop(node_b, tb3);
  EXPECT_EQ(estimator.a_timestamps().size(), 1u);
  EXPECT_EQ(estimator.b_timestamps().size(), 1u);
  EXPECT_EQ(estimator.fit().mpq_slope(), mpq_class(0));
}

}  // namespace testing
}  // namespace message_bridge
}  // namespace aos
