#include "aos/network/multinode_timestamp_filter.h"

#include <chrono>

#include "aos/configuration.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/macros.h"
#include "aos/network/testing_time_converter.h"
#include "aos/network/timestamp_filter.h"
#include "gtest/gtest.h"

namespace aos {
namespace message_bridge {
namespace testing {

namespace chrono = std::chrono;
using aos::monotonic_clock;
using aos::logger::BootTimestamp;

// Tests solution time(s) comparison and measure of invalid / inconsistent times
TEST(TimestampProblemTest, CompareTimes) {
  const BootTimestamp e = BootTimestamp::epoch();

  // Create two sets of times, offset by 1000ns
  std::vector<BootTimestamp> time_list;
  for (int i = 0; i < 10; i++) {
    time_list.push_back(e + std::chrono::nanoseconds(i * 1000));
  }

  std::vector<BootTimestamp> times_a = {time_list.begin(),
                                        time_list.end() - 1u};
  std::vector<BootTimestamp> times_b = {time_list.begin() + 1u,
                                        time_list.end()};

  CHECK_EQ(static_cast<int>(CompareTimes(times_a, times_b)),
           static_cast<int>(TimeComparison::kBefore));

  CHECK_EQ(static_cast<int>(CompareTimes(times_b, times_a)),
           static_cast<int>(TimeComparison::kAfter));

  CHECK_EQ(static_cast<int>(CompareTimes(times_a, times_a)),
           static_cast<int>(TimeComparison::kEq));

  // Now try one of the times being min_time.
  std::vector<BootTimestamp> times_b_min = times_b;
  times_b_min[5] = BootTimestamp::min_time();

  CHECK_EQ(static_cast<int>(CompareTimes(times_a, times_b_min)),
           static_cast<int>(TimeComparison::kBefore));
  CHECK_EQ(static_cast<int>(CompareTimes(times_b_min, times_a)),
           static_cast<int>(TimeComparison::kAfter));

  // Test if one of the elements is equal
  std::vector<BootTimestamp> times_b_some_eq = times_b_min;
  times_b_some_eq[2] = times_a[2];

  CHECK_EQ(static_cast<int>(CompareTimes(times_a, times_b_some_eq)),
           static_cast<int>(TimeComparison::kInvalid));
  CHECK_EQ(static_cast<int>(CompareTimes(times_b_some_eq, times_a)),
           static_cast<int>(TimeComparison::kInvalid));

  // Test if elements are out of order
  std::vector<BootTimestamp> times_b_mixed = times_b_min;
  times_b_mixed[3] = times_a[0];

  CHECK_EQ(static_cast<int>(CompareTimes(times_a, times_b_mixed)),
           static_cast<int>(TimeComparison::kInvalid));
  CHECK_EQ(static_cast<int>(CompareTimes(times_b_mixed, times_a)),
           static_cast<int>(TimeComparison::kInvalid));

  CHECK_EQ(InvalidDistance(times_a, times_a).count(), 0);
  CHECK_EQ(InvalidDistance(times_a, times_b).count(), -1000);
  CHECK_EQ(InvalidDistance(times_b, times_a).count(), -1000);
  CHECK_EQ(InvalidDistance(times_a, times_b_min).count(), -1000);
  CHECK_EQ(InvalidDistance(times_a, times_b_some_eq).count(), 0);
  CHECK_EQ(InvalidDistance(times_b_some_eq, times_a).count(), 0);
  CHECK_EQ(InvalidDistance(times_a, times_b_mixed).count(), 3000);
  CHECK_EQ(InvalidDistance(times_b_mixed, times_a).count(), 3000);
}

// Tests that a single timestamp InterpolatedTimeConverter returns equal
// results.  1 second should be 1 second everywhere.
TEST(InterpolatedTimeConverterTest, OneTime) {
  const distributed_clock::time_point de = distributed_clock::epoch();
  const BootTimestamp me = BootTimestamp::epoch();

  TestingTimeConverter time_converter(3u);
  time_converter.AddNextTimestamp(
      de + chrono::seconds(0),
      {me + chrono::seconds(1), me + chrono::seconds(10),
       me + chrono::seconds(1000)});

  EXPECT_EQ(time_converter.FromDistributedClock(0, de - chrono::seconds(1), 0),
            me + chrono::seconds(0));
  EXPECT_EQ(time_converter.FromDistributedClock(1, de - chrono::seconds(1), 0),
            me + chrono::seconds(9));
  EXPECT_EQ(time_converter.FromDistributedClock(2, de - chrono::seconds(1), 0),
            me + chrono::seconds(999));
  EXPECT_EQ(time_converter.ToDistributedClock(0, me + chrono::seconds(0)),
            de - chrono::seconds(1));
  EXPECT_EQ(time_converter.ToDistributedClock(1, me + chrono::seconds(9)),
            de - chrono::seconds(1));
  EXPECT_EQ(time_converter.ToDistributedClock(2, me + chrono::seconds(999)),
            de - chrono::seconds(1));

  EXPECT_EQ(time_converter.FromDistributedClock(0, de, 0),
            me + chrono::seconds(1));
  EXPECT_EQ(time_converter.FromDistributedClock(1, de, 0),
            me + chrono::seconds(10));
  EXPECT_EQ(time_converter.FromDistributedClock(2, de, 0),
            me + chrono::seconds(1000));
  EXPECT_EQ(time_converter.ToDistributedClock(0, me + chrono::seconds(1)), de);
  EXPECT_EQ(time_converter.ToDistributedClock(1, me + chrono::seconds(10)), de);
  EXPECT_EQ(time_converter.ToDistributedClock(2, me + chrono::seconds(1000)),
            de);
}

// Tests that actual interpolation works as expected for multiple timestamps.
TEST(InterpolatedTimeConverterTest, Interpolation) {
  const distributed_clock::time_point de = distributed_clock::epoch();
  const BootTimestamp me = BootTimestamp::epoch();

  TestingTimeConverter time_converter(3u);
  // Test that 2 timestamps interpolate correctly.
  time_converter.AddNextTimestamp(
      de + chrono::seconds(0),
      {me + chrono::seconds(1), me + chrono::seconds(10),
       me + chrono::seconds(1000)});
  time_converter.AddNextTimestamp(
      de + chrono::seconds(1),
      {me + chrono::seconds(2), me + chrono::seconds(11),
       me + chrono::seconds(1001)});

  EXPECT_EQ(
      time_converter.FromDistributedClock(0, de + chrono::milliseconds(500), 0),
      me + chrono::milliseconds(1500));
  EXPECT_EQ(
      time_converter.FromDistributedClock(1, de + chrono::milliseconds(500), 0),
      me + chrono::milliseconds(10500));
  EXPECT_EQ(
      time_converter.FromDistributedClock(2, de + chrono::milliseconds(500), 0),
      me + chrono::milliseconds(1000500));
  EXPECT_EQ(
      time_converter.ToDistributedClock(0, me + chrono::milliseconds(1500)),
      de + chrono::milliseconds(500));
  EXPECT_EQ(
      time_converter.ToDistributedClock(1, me + chrono::milliseconds(10500)),
      de + chrono::milliseconds(500));
  EXPECT_EQ(
      time_converter.ToDistributedClock(2, me + chrono::milliseconds(1000500)),
      de + chrono::milliseconds(500));

  // And that we can interpolate between points not at the start.
  time_converter.AddNextTimestamp(
      de + chrono::seconds(2),
      {me + chrono::seconds(3) - chrono::milliseconds(2),
       me + chrono::seconds(12) - chrono::milliseconds(2),
       me + chrono::seconds(1002)});

  time_converter.AddNextTimestamp(
      de + chrono::seconds(3),
      {me + chrono::seconds(4) - chrono::milliseconds(4),
       me + chrono::seconds(13) - chrono::milliseconds(2),
       me + chrono::seconds(1003) - chrono::milliseconds(2)});

  EXPECT_EQ(time_converter.FromDistributedClock(
                0, de + chrono::milliseconds(2500), 0),
            me + chrono::milliseconds(3497));
  EXPECT_EQ(time_converter.FromDistributedClock(
                1, de + chrono::milliseconds(2500), 0),
            me + chrono::milliseconds(12498));
  EXPECT_EQ(time_converter.FromDistributedClock(
                2, de + chrono::milliseconds(2500), 0),
            me + chrono::milliseconds(1002499));
  EXPECT_EQ(
      time_converter.ToDistributedClock(0, me + chrono::milliseconds(3497)),
      de + chrono::milliseconds(2500));
  EXPECT_EQ(
      time_converter.ToDistributedClock(1, me + chrono::milliseconds(12498)),
      de + chrono::milliseconds(2500));
  EXPECT_EQ(
      time_converter.ToDistributedClock(2, me + chrono::milliseconds(1002499)),
      de + chrono::milliseconds(2500));
}

// Tests that interpolation works across reboots.
TEST(InterpolatedTimeConverterTest, RebootInterpolation) {
  const distributed_clock::time_point de = distributed_clock::epoch();
  const BootTimestamp me = BootTimestamp::epoch();
  const BootTimestamp me2{.boot = 1u, .time = monotonic_clock::epoch()};

  // LOG(FATAL) << "TODO(austin): Test ToDistributedClock too";

  TestingTimeConverter time_converter(3u);
  size_t reboot_counter = 0;
  time_converter.set_reboot_found(
      [&](distributed_clock::time_point,
          const std::vector<logger::BootTimestamp> &) { ++reboot_counter; });
  // Test that 2 timestamps interpolate correctly.
  time_converter.AddNextTimestamp(
      de + chrono::seconds(0),
      {me + chrono::seconds(1), me + chrono::seconds(10),
       me + chrono::seconds(1000)});
  time_converter.AddNextTimestamp(
      de + chrono::seconds(1),
      {me + chrono::seconds(2), me + chrono::seconds(11),
       me + chrono::seconds(1001)});
  time_converter.AddNextTimestamp(
      de + chrono::seconds(2),
      {me + chrono::seconds(3), me + chrono::seconds(12),
       me + chrono::seconds(1002)});

  time_converter.AddNextTimestamp(
      de + chrono::seconds(3),
      {me2 + chrono::seconds(4), me + chrono::seconds(13),
       me + chrono::seconds(1003)});

  time_converter.AddNextTimestamp(
      de + chrono::seconds(4),
      {me2 + chrono::seconds(5), me + chrono::seconds(14),
       me + chrono::seconds(1004)});

  EXPECT_EQ(time_converter.FromDistributedClock(
                0, de + chrono::milliseconds(2400), 0),
            me + chrono::milliseconds(3400));
  EXPECT_EQ(time_converter.FromDistributedClock(
                1, de + chrono::milliseconds(2400), 0),
            me + chrono::milliseconds(12400));
  EXPECT_EQ(time_converter.FromDistributedClock(
                2, de + chrono::milliseconds(2400), 0),
            me + chrono::milliseconds(1002400));

  EXPECT_EQ(time_converter.FromDistributedClock(
                0, de + chrono::milliseconds(2900), 0),
            me + chrono::milliseconds(3900));
  EXPECT_EQ(time_converter.FromDistributedClock(
                1, de + chrono::milliseconds(2900), 0),
            me + chrono::milliseconds(12900));
  EXPECT_EQ(time_converter.FromDistributedClock(
                2, de + chrono::milliseconds(2900), 0),
            me + chrono::milliseconds(1002900));

  EXPECT_EQ(time_converter.FromDistributedClock(
                0, de + chrono::milliseconds(3000), 0),
            me + chrono::seconds(4));
  EXPECT_EQ(time_converter.FromDistributedClock(
                0, de + chrono::milliseconds(3000), 1),
            me2 + chrono::seconds(4));

  EXPECT_EQ(time_converter.FromDistributedClock(
                0, de + chrono::milliseconds(3900), 1),
            me2 + chrono::milliseconds(4900));
  EXPECT_EQ(time_converter.FromDistributedClock(
                1, de + chrono::milliseconds(3900), 0),
            me + chrono::milliseconds(13900));
  EXPECT_EQ(time_converter.FromDistributedClock(
                2, de + chrono::milliseconds(3900), 0),
            me + chrono::milliseconds(1003900));
  EXPECT_EQ(reboot_counter, 1u);
}

// Tests that reading times before the start of our interpolation points
// explodes.
TEST(InterpolatedTimeConverterDeathTest, ReadLostTime) {
  const distributed_clock::time_point de = distributed_clock::epoch();
  const BootTimestamp me = BootTimestamp::epoch();

  constexpr auto kDefaultHistoryDuration =
      InterpolatedTimeConverter::kDefaultHistoryDuration;
  constexpr chrono::nanoseconds kDt =
      kDefaultHistoryDuration /
      (InterpolatedTimeConverter::kHistoryMinCount * 2);

  TestingTimeConverter time_converter(3u);
  time_converter.StartEqual();

  // Test that 2 timestamps interpolate correctly.
  for (int i = 0; i < InterpolatedTimeConverter::kHistoryMinCount * 4; ++i) {
    time_converter.AddMonotonic({kDt, kDt, kDt});
  }

  // Force 5 seconds to be read.
  EXPECT_EQ(
      de + kDefaultHistoryDuration / 2,
      time_converter.ToDistributedClock(0, me + kDefaultHistoryDuration / 2));
  EXPECT_EQ(me + kDefaultHistoryDuration / 2,
            time_converter.FromDistributedClock(
                0, de + kDefaultHistoryDuration / 2, 0));

  // Double check we can read things from before the start
  EXPECT_EQ(de - kDt, time_converter.ToDistributedClock(0, me - kDt));
  EXPECT_EQ(me - kDt, time_converter.FromDistributedClock(0, de - kDt, 0));

  // And at and after the origin.
  EXPECT_EQ(de, time_converter.ToDistributedClock(0, me));
  EXPECT_EQ(me, time_converter.FromDistributedClock(0, de, 0));

  EXPECT_EQ(de + chrono::milliseconds(10),
            time_converter.ToDistributedClock(0, me + kDt));
  EXPECT_EQ(me + chrono::milliseconds(10),
            time_converter.FromDistributedClock(0, de + kDt, 0));

  // Now force ourselves to forget.
  time_converter.ObserveTimePassed(de + kDefaultHistoryDuration + kDt * 3 / 2);

  // Yup, can't read the origin anymore.
  EXPECT_DEATH({ LOG(INFO) << time_converter.ToDistributedClock(0, me); },
               "forgotten");
  EXPECT_DEATH({ LOG(INFO) << time_converter.FromDistributedClock(0, de, 0); },
               "forgotten");

  // But can still read the next point.
  EXPECT_EQ(de + kDt, time_converter.ToDistributedClock(0, me + kDt));
  EXPECT_EQ(me + kDt, time_converter.FromDistributedClock(0, de + kDt, 0));
}

// Tests unity time with 1 node.
TEST(InterpolatedTimeConverterTest, SingleNodeTime) {
  const distributed_clock::time_point de = distributed_clock::epoch();
  const BootTimestamp me = BootTimestamp::epoch();

  TestingTimeConverter time_converter(1u);
  time_converter.AddNextTimestamp(de + chrono::seconds(0),
                                  {me + chrono::seconds(1)});

  EXPECT_EQ(time_converter.FromDistributedClock(0, de, 0), me);
  EXPECT_EQ(
      time_converter.FromDistributedClock(0, de + chrono::seconds(100), 0),
      me + chrono::seconds(100));

  EXPECT_TRUE(time_converter.NextTimestamp());
}

// Tests that our Newtons method solver returns consistent answers for a simple
// problem or two.  Also confirm that the residual error to the constraints
// looks sane, meaning it is centered.
TEST(TimestampProblemTest, SolveNewton) {
  FlatbufferDetachedBuffer<Node> node_a_buffer =
      JsonToFlatbuffer<Node>("{\"name\": \"test_a\"}");
  const Node *const node_a = &node_a_buffer.message();

  FlatbufferDetachedBuffer<Node> node_b_buffer =
      JsonToFlatbuffer<Node>("{\"name\": \"test_b\"}");
  const Node *const node_b = &node_b_buffer.message();

  const BootTimestamp e{0, monotonic_clock::epoch()};
  const BootTimestamp ta = e + chrono::milliseconds(500);

  // Setup a time problem with an interesting shape that isn't simple and
  // parallel.
  NoncausalTimestampFilter a(node_a, node_b);
  a.Sample(e, {0, chrono::milliseconds(1000)});
  a.Sample(e + chrono::milliseconds(1000), {0, chrono::milliseconds(999)});
  a.Sample(e + chrono::milliseconds(3000), {0, chrono::milliseconds(997)});

  NoncausalTimestampFilter b(node_b, node_a);
  b.Sample(e + chrono::milliseconds(1000), {0, -chrono::milliseconds(1001)});
  b.Sample(e + chrono::milliseconds(2000), {0, -chrono::milliseconds(1002)});
  b.Sample(e + chrono::milliseconds(4000), {0, -chrono::milliseconds(1004)});

  TimestampProblem problem(2);
  problem.set_base_clock(0, ta);
  problem.set_base_clock(1, e);
  problem.add_clock_offset_filter(0, &a, 1, &b);
  problem.add_clock_offset_filter(1, &b, 0, &a);

  problem.Debug();

  problem.set_base_clock(0, e + chrono::seconds(1));
  problem.set_base_clock(1, e);

  std::vector<BootTimestamp> points1(problem.size(), BootTimestamp::max_time());
  points1[0] = e + chrono::seconds(1);

  std::tuple<std::vector<BootTimestamp>, size_t> result1 =
      problem.SolveNewton(points1);
  EXPECT_EQ(std::get<1>(result1), 0u);
  EXPECT_TRUE(problem.ValidateSolution(std::get<0>(result1)));

  std::vector<BootTimestamp> points2(problem.size(), BootTimestamp::max_time());
  points2[1] = std::get<0>(result1)[1];
  std::tuple<std::vector<BootTimestamp>, size_t> result2 =
      problem.SolveNewton(points2);
  EXPECT_EQ(std::get<1>(result2), 1u);
  EXPECT_TRUE(problem.ValidateSolution(std::get<0>(result2)));

  EXPECT_EQ(std::get<0>(result1)[0], e + chrono::seconds(1));
  EXPECT_EQ(std::get<0>(result1)[0], std::get<0>(result2)[0]);
  EXPECT_EQ(std::get<0>(result1)[1], std::get<0>(result2)[1]);

  // Confirm that the error is almost equal for both directions.  The solution
  // is an integer solution, so there will be a little bit of error left over.
  EXPECT_NEAR(a.OffsetError(std::get<0>(result1)[0], 0.0,
                            std::get<0>(result1)[1], 0.0) -
                  b.OffsetError(std::get<0>(result1)[1], 0.0,
                                std::get<0>(result1)[0], 0.0),
              0.0, 0.5);
}

}  // namespace testing
}  // namespace message_bridge
}  // namespace aos
