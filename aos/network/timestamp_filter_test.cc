#include "aos/network/timestamp_filter.h"

#include <chrono>

#include "aos/configuration.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/macros.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace aos {
namespace message_bridge {
namespace testing {

namespace chrono = std::chrono;
using aos::monotonic_clock;
using logger::BootDuration;
using logger::BootTimestamp;

class TestingNoncausalTimestampFilter : public NoncausalTimestampFilter {
 public:
  TestingNoncausalTimestampFilter(const Node *node_a, const Node *node_b)
      : NoncausalTimestampFilter(node_a, node_b) {}

  bool frozen(size_t index) const { return filter(0, 0)->frozen(index); }
  bool frozen(logger::BootTimestamp t) const {
    return filter(t.boot, 0)->frozen(t.time);
  }

  std::tuple<BootTimestamp, BootDuration> timestamp(size_t i) const {
    std::tuple<aos::monotonic_clock::time_point, std::chrono::nanoseconds>
        result = filter(0, 0)->timestamp(i);
    return std::make_tuple(BootTimestamp{0, std::get<0>(result)},
                           BootDuration{0, std::get<1>(result)});
  }
};

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

class FilterTest : public ::testing::Test {
 public:
  FlatbufferDetachedBuffer<Node> node_a_buffer =
      JsonToFlatbuffer<Node>("{\"name\": \"test_a\"}");
  const Node *const node_a = &node_a_buffer.message();

  FlatbufferDetachedBuffer<Node> node_b_buffer =
      JsonToFlatbuffer<Node>("{\"name\": \"test_b\"}");
  const Node *const node_b = &node_b_buffer.message();
};

using NoncausalTimestampFilterTest = FilterTest;
using NoncausalTimestampFilterDeathTest = FilterTest;

// Tests that 2 samples results in the correct line between them, and the
// correct intermediate as it is being built.
TEST_F(NoncausalTimestampFilterTest, PeekPop) {
  const BootTimestamp ta{
      0, monotonic_clock::time_point(chrono::nanoseconds(100000))};
  const BootDuration oa{0, chrono::nanoseconds(1000)};
  const BootTimestamp tb{
      0, monotonic_clock::time_point(chrono::nanoseconds(200000))};
  const BootDuration ob{0, chrono::nanoseconds(1100)};
  const BootTimestamp tc{
      0, monotonic_clock::time_point(chrono::nanoseconds(300000))};
  const BootDuration oc{0, chrono::nanoseconds(1010)};

  // Simple case, everything is done in order, nothing is dropped.
  {
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, oa);
    filter.Sample(tb, ob);
    filter.Sample(tc, oc);

    EXPECT_TRUE(filter.Observe());
    EXPECT_EQ(*filter.Consume(), std::make_tuple(ta, oa));
    EXPECT_TRUE(filter.Observe());
    EXPECT_EQ(*filter.Consume(), std::make_tuple(tb, ob));
    EXPECT_TRUE(filter.Observe());
    EXPECT_EQ(*filter.Consume(), std::make_tuple(tc, oc));
    EXPECT_FALSE(filter.Observe());
  }

  // Now try again while dropping ta after popping it.
  {
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, oa);
    filter.Sample(tb, ob);
    filter.Sample(tc, oc);

    EXPECT_TRUE(filter.Observe());
    EXPECT_EQ(*filter.Consume(), std::make_tuple(ta, oa));

    filter.Pop(tb);
    EXPECT_TRUE(filter.Observe());
    EXPECT_EQ(*filter.Consume(), std::make_tuple(tb, ob));
    EXPECT_TRUE(filter.Observe());
    EXPECT_EQ(*filter.Consume(), std::make_tuple(tc, oc));
    EXPECT_FALSE(filter.Observe());
  }

  // Now try again while dropping ta before popping it.
  {
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, oa);
    filter.Sample(tb, ob);
    filter.Sample(tc, oc);

    filter.Pop(tb);
    EXPECT_TRUE(filter.Observe());
    EXPECT_EQ(*filter.Consume(), std::make_tuple(tb, ob));
    EXPECT_TRUE(filter.Observe());
    EXPECT_EQ(*filter.Consume(), std::make_tuple(tc, oc));
    EXPECT_FALSE(filter.Observe());
  }
}

// Tests that invalid samples get clipped as expected.
TEST_F(NoncausalTimestampFilterTest, ClippedSample) {
  const BootTimestamp ta{0,
                         monotonic_clock::time_point(chrono::milliseconds(0))};
  const BootTimestamp tb{0,
                         monotonic_clock::time_point(chrono::milliseconds(1))};
  const BootTimestamp tc{0,
                         monotonic_clock::time_point(chrono::milliseconds(2))};

  {
    // A positive slope of 1 ms/second is properly applied.
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, {0, chrono::microseconds(1)});
    filter.Debug();
    filter.Sample(tb, {0, chrono::microseconds(2)});
    filter.Debug();
    ASSERT_EQ(filter.timestamps_size(), 2u);

    EXPECT_EQ(filter.timestamp(0),
              std::make_tuple(ta, BootDuration{0, chrono::microseconds(1)}));
    EXPECT_EQ(filter.timestamp(1),
              std::make_tuple(tb, BootDuration{0, chrono::microseconds(2)}));
  }

  {
    // A negative slope of 1 ms/second is properly applied.
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, {0, chrono::microseconds(1)});
    filter.Debug();
    filter.Sample(tb, {0, chrono::microseconds(0)});
    filter.Debug();
    ASSERT_EQ(filter.timestamps_size(), 2u);

    EXPECT_EQ(filter.timestamp(0),
              std::make_tuple(ta, BootDuration{0, chrono::microseconds(1)}));
    EXPECT_EQ(filter.timestamp(1),
              std::make_tuple(tb, BootDuration{0, chrono::microseconds(0)}));
  }

  {
    // Too much negative is ignored.
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, {0, chrono::microseconds(1)});
    filter.Debug();
    filter.Sample(tb, {0, -chrono::microseconds(1)});
    filter.Debug();
    ASSERT_EQ(filter.timestamps_size(), 1u);
  }

  {
    // Too much positive pulls up the first point.
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, {0, chrono::microseconds(1)});
    filter.Debug();
    filter.Sample(tb, {0, chrono::microseconds(3)});
    filter.Debug();
    ASSERT_EQ(filter.timestamps_size(), 2u);

    EXPECT_EQ(std::get<1>(filter.timestamp(0)),
              (BootDuration{0, chrono::microseconds(2)}));
    EXPECT_EQ(std::get<1>(filter.timestamp(1)),
              (BootDuration{0, chrono::microseconds(3)}));
  }

  {
    // Too much positive slope removes points.
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, {0, chrono::microseconds(1)});
    filter.Debug();
    filter.Sample(tb, {0, chrono::microseconds(1)});
    filter.Debug();
    ASSERT_EQ(filter.timestamps_size(), 2u);

    // Now add a sample with a slope of 0.002.  This should back propagate and
    // remove the middle point since it violates our constraints.
    filter.Sample(tc, {0, chrono::microseconds(3)});
    filter.Debug();
    ASSERT_EQ(filter.timestamps_size(), 2u);

    EXPECT_EQ(std::get<1>(filter.timestamp(0)),
              (BootDuration{0, chrono::microseconds(1)}));
    EXPECT_EQ(std::get<1>(filter.timestamp(1)),
              (BootDuration{0, chrono::microseconds(3)}));
  }
}

// Tests that removing points from the filter works as expected.
TEST_F(NoncausalTimestampFilterTest, PointRemoval) {
  const logger::BootTimestamp t_before{
      0, monotonic_clock::time_point(-chrono::milliseconds(1))};
  const BootTimestamp ta{0,
                         monotonic_clock::time_point(chrono::milliseconds(0))};
  const BootTimestamp tb{0,
                         monotonic_clock::time_point(chrono::milliseconds(1))};
  const BootTimestamp tc{0,
                         monotonic_clock::time_point(chrono::milliseconds(2))};

  // A positive slope of 1 ms/second is properly applied.
  TestingNoncausalTimestampFilter filter(node_a, node_b);

  filter.Sample(ta, {0, chrono::microseconds(1)});
  filter.Debug();
  filter.Sample(tb, {0, chrono::microseconds(2)});
  filter.Debug();
  filter.Sample(tc, {0, chrono::microseconds(1)});
  filter.Debug();
  ASSERT_EQ(filter.timestamps_size(), 3u);

  // Before or in the middle of the first line segment shouldn't change the
  // number of points.
  EXPECT_FALSE(filter.Pop(t_before));
  ASSERT_EQ(filter.timestamps_size(), 3u);

  EXPECT_FALSE(filter.Pop(ta));
  ASSERT_EQ(filter.timestamps_size(), 3u);

  EXPECT_FALSE(filter.Pop(ta + chrono::microseconds(100)));
  ASSERT_EQ(filter.timestamps_size(), 3u);

  // The second point should trigger a pop, since the offset computed using the
  // points won't change when it is used, and any times after (even 1-2 ns
  // later) would be wrong.
  EXPECT_TRUE(filter.Pop(tb));
  ASSERT_EQ(filter.timestamps_size(), 2u);
}

// Tests that inserting duplicate points causes the duplicates to get ignored.
TEST_F(NoncausalTimestampFilterTest, DuplicatePoints) {
  const BootTimestamp ta{0,
                         monotonic_clock::time_point(chrono::milliseconds(0))};
  const BootDuration oa{0, chrono::microseconds(1)};
  const BootTimestamp tb{0,
                         monotonic_clock::time_point(chrono::milliseconds(1))};
  const BootDuration ob{0, chrono::microseconds(2)};

  TestingNoncausalTimestampFilter filter(node_a, node_b);

  filter.Sample(ta, oa);
  filter.Sample(tb, ob);
  EXPECT_EQ(filter.timestamps_size(), 2u);

  filter.Sample(tb, ob);
  EXPECT_EQ(filter.timestamps_size(), 2u);
}

// Tests that inserting points in the middle of the time sequence works for the
// simple case.
TEST_F(NoncausalTimestampFilterTest, BackwardsInTimeSimple) {
  // Start with the simple case.  A valid point in the middle.
  const BootTimestamp ta{0,
                         monotonic_clock::time_point(chrono::milliseconds(0))};
  const BootDuration oa{0, chrono::microseconds(1)};

  const BootTimestamp tb{0,
                         monotonic_clock::time_point(chrono::milliseconds(1))};
  const BootDuration ob{0, chrono::microseconds(0)};

  const BootTimestamp tc{0,
                         monotonic_clock::time_point(chrono::milliseconds(2))};
  const BootDuration oc{0, chrono::microseconds(1)};
  TestingNoncausalTimestampFilter filter(node_a, node_b);

  filter.Sample(ta, oa);
  filter.Sample(tc, oc);
  filter.Sample(tb, ob);
  filter.Debug();
  EXPECT_EQ(filter.timestamps_size(), 3u);

  EXPECT_EQ(filter.timestamp(0), std::make_tuple(ta, oa));
  EXPECT_EQ(filter.timestamp(1), std::make_tuple(tb, ob));
  EXPECT_EQ(filter.timestamp(2), std::make_tuple(tc, oc));
}

// Tests that inserting a duplicate point at the beginning gets ignored if it is
// more negative than the original beginning point.
TEST_F(NoncausalTimestampFilterTest, BackwardsInTimeDuplicateNegative) {
  const BootTimestamp ta{0,
                         monotonic_clock::time_point(chrono::milliseconds(0))};
  const BootDuration oa{0, chrono::microseconds(1)};

  const BootTimestamp tb{0,
                         monotonic_clock::time_point(chrono::milliseconds(1))};
  const BootDuration ob{0, chrono::microseconds(1)};
  TestingNoncausalTimestampFilter filter(node_a, node_b);

  filter.Sample(ta, oa);
  filter.Sample(tb, ob);
  filter.Sample(ta, {0, chrono::microseconds(0)});
  filter.Debug();
  EXPECT_EQ(filter.timestamps_size(), 2u);

  EXPECT_EQ(filter.timestamp(0), std::make_tuple(ta, oa));
  EXPECT_EQ(filter.timestamp(1), std::make_tuple(tb, ob));
}

// Tests that inserting a better duplicate point at the beginning gets taken if
// it is more positive than the original beginning point.
TEST_F(NoncausalTimestampFilterTest, BackwardsInTimeDuplicatePositive) {
  const BootTimestamp ta{0,
                         monotonic_clock::time_point(chrono::milliseconds(0))};
  const BootDuration oa{0, chrono::microseconds(1)};

  const BootTimestamp tb{0,
                         monotonic_clock::time_point(chrono::milliseconds(1))};
  const BootDuration ob{0, chrono::microseconds(1)};
  TestingNoncausalTimestampFilter filter(node_a, node_b);

  filter.Sample(ta, oa);
  filter.Sample(tb, ob);
  filter.Sample(ta, {0, chrono::microseconds(2)});
  filter.Debug();
  EXPECT_EQ(filter.timestamps_size(), 2u);

  EXPECT_EQ(filter.timestamp(0),
            std::make_tuple(ta, BootDuration{0, chrono::microseconds(2)}));
  EXPECT_EQ(filter.timestamp(1), std::make_tuple(tb, ob));
}

// Tests that inserting a negative duplicate point in the middle is dropped.
TEST_F(NoncausalTimestampFilterTest, BackwardsInTimeMiddleDuplicateNegative) {
  const BootTimestamp ta{0,
                         monotonic_clock::time_point(chrono::milliseconds(0))};
  const BootDuration oa{0, chrono::microseconds(1)};

  const BootTimestamp tb{0,
                         monotonic_clock::time_point(chrono::milliseconds(1))};
  const BootDuration ob{0, chrono::microseconds(2)};

  const BootTimestamp tc{0,
                         monotonic_clock::time_point(chrono::milliseconds(2))};
  const BootDuration oc{0, chrono::microseconds(1)};
  TestingNoncausalTimestampFilter filter(node_a, node_b);

  filter.Sample(ta, oa);
  filter.Sample(tb, ob);
  filter.Sample(tc, oc);
  filter.Sample(tb, {0, chrono::microseconds(0)});
  filter.Debug();
  EXPECT_EQ(filter.timestamps_size(), 3u);

  EXPECT_EQ(filter.timestamp(0), std::make_tuple(ta, oa));
  EXPECT_EQ(filter.timestamp(1), std::make_tuple(tb, ob));
  EXPECT_EQ(filter.timestamp(2), std::make_tuple(tc, oc));
}

// Tests that inserting a positive duplicate point in the middle is taken.
TEST_F(NoncausalTimestampFilterTest, BackwardsInTimeMiddleDuplicatePositive) {
  const BootTimestamp ta{0,
                         monotonic_clock::time_point(chrono::milliseconds(0))};
  const BootDuration oa{0, chrono::microseconds(1)};

  const BootTimestamp tb{0,
                         monotonic_clock::time_point(chrono::milliseconds(1))};
  const BootDuration ob{0, chrono::microseconds(0)};

  const BootTimestamp tc{0,
                         monotonic_clock::time_point(chrono::milliseconds(2))};
  const BootDuration oc{0, chrono::microseconds(1)};
  TestingNoncausalTimestampFilter filter(node_a, node_b);

  filter.Sample(ta, oa);
  filter.Sample(tb, ob);
  filter.Sample(tc, oc);
  filter.Sample(tb, {0, chrono::microseconds(2)});
  filter.Debug();
  EXPECT_EQ(filter.timestamps_size(), 3u);

  EXPECT_EQ(filter.timestamp(0), std::make_tuple(ta, oa));
  EXPECT_EQ(filter.timestamp(1),
            std::make_tuple(tb, BootDuration{0, chrono::microseconds(2)}));
  EXPECT_EQ(filter.timestamp(2), std::make_tuple(tc, oc));
}

// Tests that a bunch of points added in any order results in the same answer as
// adding them in order.  4 points should give us enough combos, and try all the
// orderings of the points too.  Given that the in and out of order code
// essentially re-implements the same logic, this is an awesome consistency
// check.
TEST_F(NoncausalTimestampFilterTest, RandomTimeInsertion) {
  // 2 ms apart with 1 us of resolution on the delta should give us all sorts of
  // equality constraints for all sorts of orderings.
  const std::array<monotonic_clock::time_point, 4> t(
      {monotonic_clock::time_point(chrono::milliseconds(0)),
       monotonic_clock::time_point(chrono::milliseconds(2)),
       monotonic_clock::time_point(chrono::milliseconds(4)),
       monotonic_clock::time_point(chrono::milliseconds(6))});

  for (int i = -10; i < 10; ++i) {
    for (int j = -10; j < 10; ++j) {
      for (int k = -10; k < 10; ++k) {
        for (int l = -10; l < 10; ++l) {
          std::array<chrono::nanoseconds, 4> o(
              {chrono::microseconds(i), chrono::microseconds(j),
               chrono::microseconds(k), chrono::microseconds(l)});
          TestingNoncausalTimestampFilter forward(node_a, node_b);

          VLOG(1) << "Sorting in order";
          forward.Sample({0, t[0]}, {0, o[0]});
          forward.Sample({0, t[1]}, {0, o[1]});
          forward.Sample({0, t[2]}, {0, o[2]});
          forward.Sample({0, t[3]}, {0, o[3]});

          // Confirm everything is within the velocity bounds.
          for (size_t i = 1; i < forward.timestamps_size(); ++i) {
            const chrono::nanoseconds dt =
                std::get<0>(forward.timestamp(i)).time -
                std::get<0>(forward.timestamp(i - 1)).time;
            const chrono::nanoseconds doffset =
                std::get<1>(forward.timestamp(i)).duration -
                std::get<1>(forward.timestamp(i - 1)).duration;
            EXPECT_GE(doffset, -dt * kMaxVelocity());
            EXPECT_LE(doffset, dt * kMaxVelocity());
          }

          // Now that we have the correct answer, try all the combos to see
          // what breaks it.
          std::array<int, 4> indices({0, 1, 2, 3});
          int r = 0;
          do {
            std::array<
                std::pair<monotonic_clock::time_point, chrono::nanoseconds>, 4>
                pairs({std::make_pair(t[indices[0]], o[indices[0]]),
                       std::make_pair(t[indices[1]], o[indices[1]]),
                       std::make_pair(t[indices[2]], o[indices[2]]),
                       std::make_pair(t[indices[3]], o[indices[3]])});

            VLOG(1) << "Sorting randomized";
            TestingNoncausalTimestampFilter random(node_a, node_b);
            random.Sample({0, pairs[0].first}, {0, pairs[0].second});
            if (VLOG_IS_ON(1)) {
              random.Debug();
            }
            random.Sample({0, pairs[1].first}, {0, pairs[1].second});
            if (VLOG_IS_ON(1)) {
              random.Debug();
            }
            random.Sample({0, pairs[2].first}, {0, pairs[2].second});
            if (VLOG_IS_ON(1)) {
              random.Debug();
            }
            random.Sample({0, pairs[3].first}, {0, pairs[3].second});
            if (VLOG_IS_ON(1)) {
              random.Debug();
            }

            if (forward.timestamps_size() != random.timestamps_size()) {
              LOG(INFO) << "Iteration i == " << i << " && j == " << j
                        << " && k == " << k << " && l == " << l
                        << " && r == " << r;
              LOG(INFO) << "Forward";
              forward.Debug();
              LOG(INFO) << "Random";
              for (int i = 0; i < 4; ++i) {
                LOG(INFO) << "Sample({0, " << pairs[i].first << "}, "
                          << pairs[i].second.count() << ")";
              }
              random.Debug();
            }
            ASSERT_EQ(forward.timestamps_size(), random.timestamps_size());
            for (size_t s = 0; s < forward.timestamps_size(); ++s) {
              EXPECT_EQ(forward.timestamp(s), random.timestamp(s));
            }
            ++r;
          } while (std::next_permutation(indices.begin(), indices.end()));
        }
      }
    }
  }
}

// Tests that the right points get frozen when we ask for them to be.
TEST_F(NoncausalTimestampFilterTest, FrozenTimestamps) {
  // Start with the simple case.  A valid point in the middle.
  const BootTimestamp ta{0,
                         monotonic_clock::time_point(chrono::milliseconds(0))};
  const BootDuration oa{0, chrono::microseconds(1)};

  const BootTimestamp tb{0,
                         monotonic_clock::time_point(chrono::milliseconds(1))};
  const BootDuration ob{0, chrono::microseconds(0)};

  const BootTimestamp tc{0,
                         monotonic_clock::time_point(chrono::milliseconds(2))};
  const BootDuration oc{0, chrono::microseconds(1)};

  // Test for our node.
  {
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, oa);
    filter.Sample(tc, oc);
    filter.Sample(tb, ob);
    ASSERT_EQ(filter.timestamps_size(), 3u);

    filter.FreezeUntil(ta - chrono::microseconds(1),
                       {0, monotonic_clock::min_time});
    EXPECT_TRUE(filter.frozen(0));
    EXPECT_FALSE(filter.frozen(1));

    filter.FreezeUntil(ta, {0, monotonic_clock::min_time});
    EXPECT_TRUE(filter.frozen(0));
    EXPECT_FALSE(filter.frozen(1));

    filter.FreezeUntil(ta + chrono::microseconds(1),
                       {0, monotonic_clock::min_time});
    EXPECT_TRUE(filter.frozen(0));
    EXPECT_TRUE(filter.frozen(1));
    EXPECT_FALSE(filter.frozen(2));

    filter.FreezeUntil(tc, {0, monotonic_clock::min_time});
    EXPECT_TRUE(filter.frozen(0));
    EXPECT_TRUE(filter.frozen(1));
    EXPECT_TRUE(filter.frozen(2));
  }

  // Test that fully frozen doesn't apply when there is 1 time and we are before
  // the start.
  {
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, oa);

    filter.FreezeUntil(ta - chrono::microseconds(1),
                       {0, monotonic_clock::min_time});
    EXPECT_TRUE(filter.frozen(0));

    // New samples aren't frozen until they are explicitly frozen.
    filter.Sample(tb, ob);
    EXPECT_FALSE(filter.frozen(1));
    filter.FreezeUntil(ta + chrono::microseconds(1),
                       {0, monotonic_clock::min_time});

    EXPECT_TRUE(filter.frozen(1));
  }

  // Test the remote node
  {
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, oa);
    filter.Sample(tc, oc);
    filter.Sample(tb, ob);
    ASSERT_EQ(filter.timestamps_size(), 3u);

    // Trigger FreezeUntilRemote
    filter.FreezeUntil({0, monotonic_clock::min_time},
                       ta + oa.duration - chrono::microseconds(1));
    EXPECT_TRUE(filter.frozen(0));
    EXPECT_FALSE(filter.frozen(1));

    filter.FreezeUntil({0, monotonic_clock::min_time}, ta + oa);
    EXPECT_TRUE(filter.frozen(0));
    EXPECT_FALSE(filter.frozen(1));

    filter.FreezeUntil({0, monotonic_clock::min_time},
                       ta + oa.duration + chrono::microseconds(1));
    EXPECT_TRUE(filter.frozen(0));
    EXPECT_TRUE(filter.frozen(1));
    EXPECT_FALSE(filter.frozen(2));

    filter.FreezeUntil({0, monotonic_clock::min_time}, tc + oc);
    EXPECT_TRUE(filter.frozen(0));
    EXPECT_TRUE(filter.frozen(1));
    EXPECT_TRUE(filter.frozen(2));
  }

  // Test that fully frozen doesn't apply when there is 1 time and we are before
  // the start on the remote node.
  {
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, oa);

    // Trigger FreezeUntilRemote
    filter.FreezeUntil({0, monotonic_clock::min_time},
                       ta + oa.duration - chrono::microseconds(1));
    EXPECT_TRUE(filter.frozen(0));

    filter.Sample(tb, ob);
    EXPECT_FALSE(filter.frozen(1));
    filter.FreezeUntil({0, monotonic_clock::min_time},
                       ta + oa.duration + chrono::microseconds(1));

    EXPECT_TRUE(filter.frozen(1));
  }
}

// Tests that we refuse to modify frozen points in a bunch of different ways.
TEST_F(NoncausalTimestampFilterDeathTest, FrozenTimestamps) {
  // Start with the simple case.  A valid point in the middle.
  const BootTimestamp ta{0,
                         monotonic_clock::time_point(chrono::milliseconds(0))};
  const BootDuration oa{0, chrono::microseconds(100)};

  const BootTimestamp tb{
      0, monotonic_clock::time_point(chrono::milliseconds(100))};
  const BootDuration ob{0, chrono::microseconds(0)};

  const BootTimestamp tc{
      0, monotonic_clock::time_point(chrono::milliseconds(200))};
  const BootDuration oc{0, chrono::microseconds(100)};

  {
    // Test that adding before a frozen sample explodes.
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, oa);
    filter.Sample(tb, ob);
    ASSERT_EQ(filter.timestamps_size(), 2u);
    filter.FreezeUntil(tb, {0, monotonic_clock::min_time});

    EXPECT_DEATH(
        {
          filter.Sample(tb, oa);
        },
        "monotonic_now > frozen_time_ \\(0.100000000sec vs. "
        "0.100000000sec\\) : test_a -> test_b Tried to insert "
        "0.100000000sec before the frozen time of 0.100000000sec.  "
        "Increase --time_estimation_buffer_seconds to greater than 0");
  }

  {
    // Test that if we freeze it all after the end, we refuse any new samples.
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, oa);
    filter.Sample(tb, ob);
    ASSERT_EQ(filter.timestamps_size(), 2u);
    filter.FreezeUntil(tc, {0, monotonic_clock::min_time});

    EXPECT_DEATH(
        {
          filter.Sample(tc, oc);
        },
        "test_a -> test_b Returned a horizontal line previously and then got a "
        "new sample at "
        "0.200000000sec, 0.2 seconds after the last sample at 0.000000000sec");
  }

  {
    // Test that if we freeze it all after the end, we refuse any new samples.
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, oa);
    filter.Sample(tc, oc);
    ASSERT_EQ(filter.timestamps_size(), 2u);
    filter.FreezeUntil(tc, {0, monotonic_clock::min_time});

    EXPECT_DEATH(
        {
          filter.Sample(tb, ob);
        },
        "monotonic_now > frozen_time_ \\(0.100000000sec vs. "
        "0.200000000sec\\) : test_a -> test_b Tried to insert "
        "0.100000000sec before the frozen time of 0.200000000sec.  "
        "Increase --time_estimation_buffer_seconds to greater than 0.1");
  }

  {
    // Test that if we freeze, and a point in the middle triggers back
    // propagation, we refuse.
    TestingNoncausalTimestampFilter filter(node_a, node_b);

    filter.Sample(ta, oa);
    filter.Sample(tb, ob);
    filter.Sample(tc, oc);
    ASSERT_EQ(filter.timestamps_size(), 3u);
    filter.FreezeUntil(tb, {0, monotonic_clock::min_time});

    EXPECT_DEATH({ filter.Sample(tb, oa); },
                 "monotonic_now > frozen_time_ \\(0.100000000sec vs. "
                 "0.100000000sec\\) : test_a -> test_b Tried to insert "
                 "0.100000000sec before the frozen time of 0.100000000sec.  "
                 "Increase --time_estimation_buffer_seconds to greater than 0");
    EXPECT_DEATH({ filter.Sample(tb + chrono::nanoseconds(1), oa); },
                 "test_a -> test_b Can't pop an already frozen sample");
  }
}

// Tests that all variants of InterpolateOffset do reasonable things.
TEST_F(NoncausalTimestampFilterTest, InterpolateOffset) {
  const monotonic_clock::time_point e = monotonic_clock::epoch();

  const monotonic_clock::time_point t1 = e + chrono::nanoseconds(10000);
  const chrono::nanoseconds o1 = chrono::nanoseconds(100);
  const double o1d = static_cast<double>(o1.count());

  const monotonic_clock::time_point t2 = t1 + chrono::nanoseconds(1000);
  const chrono::nanoseconds o2 = chrono::nanoseconds(150);
  const double o2d = static_cast<double>(o2.count());

  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffset(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2), t1),
            o1);
  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffset(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2), t1, 0.0),
            o1);
  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffsetRemainder(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2), t1, 0.0),
            0.0);

  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffset(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2), t2),
            o2);
  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffset(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2), t2, 0.0),
            o1);
  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffsetRemainder(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2), t2, 0.0),
            o2d - o1d);

  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffset(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2),
                t1 + chrono::nanoseconds(500)),
            chrono::nanoseconds(125));
  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffset(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2),
                t1 + chrono::nanoseconds(500), 0.0),
            o1);
  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffsetRemainder(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2),
                t1 + chrono::nanoseconds(500), 0.0),
            25.);

  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffset(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2),
                t1 + chrono::nanoseconds(-200)),
            chrono::nanoseconds(90));
  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffset(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2), t1, -200.),
            o1);
  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffsetRemainder(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2), t1, -200.),
            -10.);

  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffset(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2),
                t1 + chrono::nanoseconds(200)),
            chrono::nanoseconds(110));
  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffset(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2), t1, 200.),
            o1);
  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffsetRemainder(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2), t1, 200.),
            10.);

  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffset(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2),
                t1 + chrono::nanoseconds(800)),
            chrono::nanoseconds(140));
  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffset(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2), t1, 800.),
            o1);
  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffsetRemainder(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2), t1, 800.),
            40.);

  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffset(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2),
                t1 + chrono::nanoseconds(1200)),
            chrono::nanoseconds(160));
  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffset(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2), t1, 1200.),
            o1);
  EXPECT_EQ(NoncausalTimestampFilter::InterpolateOffsetRemainder(
                std::make_tuple(t1, o1), std::make_tuple(t2, o2), t1, 1200.),
            60.);

  // Test extrapolation functions before t1 and after t2
  EXPECT_EQ(
      NoncausalTimestampFilter::ExtrapolateOffset(std::make_tuple(t1, o1), t1),
      o1);

  // With small offset (<1/kMaxVelocity() sec), low precision can't
  // tell the difference since the delta is a fraction of a nanosecond
  EXPECT_EQ(NoncausalTimestampFilter::ExtrapolateOffset(
                std::make_tuple(t1, o1), t1 + chrono::nanoseconds(-800)),
            o1);

  EXPECT_EQ(NoncausalTimestampFilter::ExtrapolateOffset(
                std::make_tuple(t1, o1), e + chrono::nanoseconds(1000)),
            o1 - chrono::nanoseconds(
                     int64_t((t1 - (e + chrono::nanoseconds(1000))).count() *
                             kMaxVelocity())));

  EXPECT_EQ(NoncausalTimestampFilter::ExtrapolateOffset(
                std::make_tuple(t1, o1), t1 + chrono::nanoseconds(-9000)),
            o1 + chrono::nanoseconds(int64_t(
                     chrono::nanoseconds(-9000).count() * kMaxVelocity())));

  // Test base + double version
  EXPECT_EQ(NoncausalTimestampFilter::ExtrapolateOffset(std::make_tuple(t1, o1),
                                                        e, 0.),
            o1);
  EXPECT_EQ(NoncausalTimestampFilter::ExtrapolateOffsetRemainder(
                std::make_tuple(t1, o1), e, 0.),
            -(t1 - e).count() * kMaxVelocity());

  EXPECT_EQ(NoncausalTimestampFilter::ExtrapolateOffset(std::make_tuple(t1, o1),
                                                        t1, 0.),
            o1);
  EXPECT_EQ(NoncausalTimestampFilter::ExtrapolateOffsetRemainder(
                std::make_tuple(t1, o1), t1, 0.),
            0.);

  EXPECT_EQ(NoncausalTimestampFilter::ExtrapolateOffset(std::make_tuple(t1, o1),
                                                        t1, -1000.),
            o1);
  EXPECT_EQ(NoncausalTimestampFilter::ExtrapolateOffsetRemainder(
                std::make_tuple(t1, o1), t1, -1000.),
            -1000. * kMaxVelocity());

  EXPECT_EQ(
      NoncausalTimestampFilter::ExtrapolateOffset(std::make_tuple(t2, o2), t2),
      o2);

  EXPECT_EQ(NoncausalTimestampFilter::ExtrapolateOffset(std::make_tuple(t2, o2),
                                                        t2, 0.0),
            o2);
  EXPECT_EQ(NoncausalTimestampFilter::ExtrapolateOffsetRemainder(
                std::make_tuple(t2, o2), t2, 0.0),
            0.0);

  // Test points past our last sample
  EXPECT_EQ(NoncausalTimestampFilter::ExtrapolateOffset(
                std::make_tuple(t2, o2), t2 + chrono::nanoseconds(10000)),
            chrono::nanoseconds(
                static_cast<int64_t>(o2.count() - 10000. * kMaxVelocity())));

  EXPECT_EQ(NoncausalTimestampFilter::ExtrapolateOffset(std::make_tuple(t2, o2),
                                                        t2, 100.0),
            o2);
  EXPECT_EQ(NoncausalTimestampFilter::ExtrapolateOffsetRemainder(
                std::make_tuple(t2, o2), t2, 100.0),
            -100.0 * kMaxVelocity());
}

// Tests that FindTimestamps finds timestamps in a sequence.
TEST_F(NoncausalTimestampFilterTest, FindTimestamps) {
  const BootTimestamp e{0, monotonic_clock::epoch()};
  // Note: t1, t2, t3 need to be picked such that the slop is small so filter
  // doesn't modify the timestamps.
  const BootTimestamp t1 = e + chrono::nanoseconds(0);
  const BootDuration o1{0, chrono::nanoseconds(100)};
  const BootTimestamp t2 = e + chrono::microseconds(1000);
  const BootDuration o2{0, chrono::nanoseconds(150)};
  const BootTimestamp t3 = e + chrono::microseconds(2000);
  const BootDuration o3{0, chrono::nanoseconds(50)};

  TestingNoncausalTimestampFilter filter(node_a, node_b);

  filter.Sample(t1, o1);
  filter.Sample(t2, o2);
  filter.Sample(t3, o3);

  // Try points before, after, and at each of the points in the line.
  EXPECT_THAT(filter.FindTimestamps(e - chrono::microseconds(10), 0),
              ::testing::Pair(::testing::Eq(std::make_tuple(t1, o1)),
                              ::testing::Eq(std::make_tuple(t2, o2))));
  EXPECT_THAT(filter.FindTimestamps(e - chrono::microseconds(10), 0.9, 0),
              ::testing::Pair(::testing::Eq(std::make_tuple(t1, o1)),
                              ::testing::Eq(std::make_tuple(t2, o2))));

  EXPECT_THAT(filter.FindTimestamps(e + chrono::microseconds(0), 0),
              ::testing::Pair(::testing::Eq(std::make_tuple(t1, o1)),
                              ::testing::Eq(std::make_tuple(t2, o2))));
  EXPECT_THAT(filter.FindTimestamps(e + chrono::microseconds(0), 0.8, 0),
              ::testing::Pair(::testing::Eq(std::make_tuple(t1, o1)),
                              ::testing::Eq(std::make_tuple(t2, o2))));

  EXPECT_THAT(filter.FindTimestamps(e + chrono::microseconds(100), 0),
              ::testing::Pair(::testing::Eq(std::make_tuple(t1, o1)),
                              ::testing::Eq(std::make_tuple(t2, o2))));
  EXPECT_THAT(filter.FindTimestamps(e + chrono::microseconds(100), 0.7, 0),
              ::testing::Pair(::testing::Eq(std::make_tuple(t1, o1)),
                              ::testing::Eq(std::make_tuple(t2, o2))));

  EXPECT_THAT(filter.FindTimestamps(e + chrono::microseconds(1000), 0),
              ::testing::Pair(::testing::Eq(std::make_tuple(t2, o2)),
                              ::testing::Eq(std::make_tuple(t3, o3))));
  EXPECT_THAT(filter.FindTimestamps(e + chrono::microseconds(1000), 0.0, 0),
              ::testing::Pair(::testing::Eq(std::make_tuple(t2, o2)),
                              ::testing::Eq(std::make_tuple(t3, o3))));

  EXPECT_THAT(filter.FindTimestamps(e + chrono::microseconds(1500), 0),
              ::testing::Pair(::testing::Eq(std::make_tuple(t2, o2)),
                              ::testing::Eq(std::make_tuple(t3, o3))));
  EXPECT_THAT(filter.FindTimestamps(e + chrono::microseconds(1500), 0.0, 0),
              ::testing::Pair(::testing::Eq(std::make_tuple(t2, o2)),
                              ::testing::Eq(std::make_tuple(t3, o3))));

  EXPECT_THAT(filter.FindTimestamps(e + chrono::microseconds(2000), 0),
              ::testing::Pair(::testing::Eq(std::make_tuple(t2, o2)),
                              ::testing::Eq(std::make_tuple(t3, o3))));
  EXPECT_THAT(filter.FindTimestamps(e + chrono::microseconds(2000), 0.1, 0),
              ::testing::Pair(::testing::Eq(std::make_tuple(t2, o2)),
                              ::testing::Eq(std::make_tuple(t3, o3))));

  EXPECT_THAT(filter.FindTimestamps(e + chrono::microseconds(2500), 0),
              ::testing::Pair(::testing::Eq(std::make_tuple(t2, o2)),
                              ::testing::Eq(std::make_tuple(t3, o3))));
  EXPECT_THAT(filter.FindTimestamps(e + chrono::microseconds(2500), 0.0, 0),
              ::testing::Pair(::testing::Eq(std::make_tuple(t2, o2)),
                              ::testing::Eq(std::make_tuple(t3, o3))));
}

// Tests that Offset returns results indicative of it calling InterpolateOffset
// and FindTimestamps correctly.
TEST_F(NoncausalTimestampFilterTest, Offset) {
  const BootTimestamp e{0, monotonic_clock::epoch()};
  // Note: t1, t2, t3 need to be picked such that the slope is small so filter
  // doesn't modify the timestamps.
  const BootTimestamp t1 = e + chrono::nanoseconds(1000);
  const BootDuration o1{0, chrono::nanoseconds(100)};
  const double o1d = static_cast<double>(o1.duration.count());

  const BootTimestamp t2 = e + chrono::microseconds(2000);
  const BootDuration o2{0, chrono::nanoseconds(150)};
  const double o2d = static_cast<double>(o2.duration.count());

  const BootTimestamp t3 = e + chrono::microseconds(3000);
  const BootDuration o3{0, chrono::nanoseconds(50)};
  const double o3d = static_cast<double>(o3.duration.count());

  const BootTimestamp t4 = e + chrono::microseconds(4000);

  TestingNoncausalTimestampFilter filter(node_a, node_b);

  filter.Sample(t1, o1);

  // 1 point is handled properly.
  EXPECT_EQ(filter.Offset(t1, 0), o1);
  EXPECT_EQ(filter.Offset(t1, 0.0, 0), std::make_pair(o1, 0.0));
  // Check if we ask for something away from point that we get an offset
  // based on the MaxVelocity allowed
  const double offset_pre = -(t1.time - e.time).count() * kMaxVelocity();
  EXPECT_EQ(filter.Offset(e, 0),
            o1 + chrono::nanoseconds(static_cast<int64_t>(offset_pre)));
  EXPECT_EQ(filter.Offset(e, 0.0, 0), std::make_pair(o1, offset_pre));

  double offset_post = -(t2.time - t1.time).count() * kMaxVelocity();
  EXPECT_EQ(filter.Offset(t2, 0),
            o1 + chrono::nanoseconds(static_cast<int64_t>(offset_post)));
  EXPECT_EQ(filter.Offset(t2, 0.0, 0), std::make_pair(o1, offset_post));

  filter.Sample(t2, o2);
  filter.Sample(t3, o3);

  EXPECT_EQ(filter.Offset(t1, 0), o1);
  EXPECT_EQ(filter.Offset(t2, 0), o2);
  EXPECT_EQ(filter.Offset(t3, 0), o3);

  EXPECT_EQ(filter.Offset(t1, 0.0, 0), std::make_pair(o1, 0.0));

  EXPECT_EQ(filter.Offset(
                e + (t2.time_since_epoch() + t1.time_since_epoch()) / 2, 0.0, 0),
            std::make_pair(o1, (o2d - o1d) / 2.));

  EXPECT_EQ(filter.Offset(t2, 0.0, 0), std::make_pair(o2, 0.0));

  EXPECT_EQ(
      filter.Offset(e + (t2.time_since_epoch() + t3.time_since_epoch()) / 2,
                    0.0, 0),
      std::make_pair(o2, (o2d + o3d) / 2. - o2d));

  EXPECT_EQ(filter.Offset(t3, 0.0, 0), std::make_pair(o3, 0.0));

  // Check that we still get same answer for times before our sample data...
  EXPECT_EQ(filter.Offset(e, 0),
            o1 + chrono::nanoseconds(static_cast<int64_t>(offset_pre)));
  EXPECT_EQ(filter.Offset(e, 0.0, 0), std::make_pair(o1, offset_pre));
  // ... and after
  offset_post = -(t4.time - t3.time).count() * kMaxVelocity();
  EXPECT_EQ(filter.Offset(t4, 0),
            (o3 + chrono::nanoseconds(static_cast<int64_t>(offset_post))));
  EXPECT_EQ(filter.Offset(t4, 0.0, 0), std::make_pair(o3, offset_post));
}

// Run a couple of points through the estimator and confirm it works.
TEST(NoncausalOffsetEstimatorTest, FullEstimator) {
  const aos::FlatbufferDetachedBuffer<Node> node_a_buffer =
      JsonToFlatbuffer<Node>("{\"name\": \"a\"}");
  const aos::FlatbufferDetachedBuffer<Node> node_b_buffer =
      JsonToFlatbuffer<Node>("{\"name\": \"b\"}");

  const Node *node_a = &node_a_buffer.message();
  const Node *node_b = &node_b_buffer.message();

  const BootTimestamp ta1{
      0, monotonic_clock::time_point(chrono::milliseconds(1000))};
  const BootTimestamp ta2 = ta1 + chrono::milliseconds(10);
  const BootTimestamp ta3 = ta1 + chrono::milliseconds(20);

  const BootTimestamp tb1{
      0, monotonic_clock::time_point(chrono::milliseconds(4000))};
  const BootTimestamp tb2 =
      tb1 + chrono::milliseconds(10) + chrono::nanoseconds(100);
  const BootTimestamp tb3 = tb1 + chrono::milliseconds(20);

  NoncausalOffsetEstimator estimator(node_a, node_b);

  // Add 3 timestamps in and confirm that the slopes come out reasonably.
  estimator.Sample(node_a, ta1, tb1);
  estimator.Sample(node_b, tb1, ta1);
  EXPECT_EQ(estimator.GetFilter(node_a)->timestamps_size(), 1u);
  EXPECT_EQ(estimator.GetFilter(node_b)->timestamps_size(), 1u);

  estimator.Sample(node_a, ta2, tb2);
  estimator.Sample(node_b, tb2, ta2);
  EXPECT_EQ(estimator.GetFilter(node_a)->timestamps_size(), 2u);
  EXPECT_EQ(estimator.GetFilter(node_b)->timestamps_size(), 2u);

  estimator.ReverseSample(node_b, tb3, ta3);
  estimator.ReverseSample(node_a, ta3, tb3);
  EXPECT_EQ(estimator.GetFilter(node_a)->timestamps_size(), 3u);
  EXPECT_EQ(estimator.GetFilter(node_b)->timestamps_size(), 3u);

  estimator.GetFilter(node_a)->Pop(ta2);
  estimator.GetFilter(node_b)->Pop(tb2);
  EXPECT_EQ(estimator.GetFilter(node_a)->timestamps_size(), 2u);
  EXPECT_EQ(estimator.GetFilter(node_b)->timestamps_size(), 2u);

  // And confirm we won't drop down to 1 point.
  estimator.GetFilter(node_a)->Pop(ta3);
  estimator.GetFilter(node_b)->Pop(tb3);
  EXPECT_EQ(estimator.GetFilter(node_a)->timestamps_size(), 2u);
  EXPECT_EQ(estimator.GetFilter(node_b)->timestamps_size(), 2u);
}

}  // namespace testing
}  // namespace message_bridge
}  // namespace aos
