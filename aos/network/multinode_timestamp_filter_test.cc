#include "aos/network/timestamp_filter.h"

#include <chrono>

#include <nlopt.h>
#include "aos/configuration.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/macros.h"
#include "aos/network/multinode_timestamp_filter.h"
#include "aos/network/testing_time_converter.h"
#include "gtest/gtest.h"

namespace aos {
namespace message_bridge {
namespace testing {

namespace chrono = std::chrono;
using aos::monotonic_clock;

mpq_class SolveExact(Line la, Line lb, monotonic_clock::time_point ta) {
  mpq_class ma = la.mpq_slope();
  mpq_class ba = la.mpq_offset();
  mpq_class mb = lb.mpq_slope();
  mpq_class bb = lb.mpq_offset();
  // The min of a quadratic is when the slope is 0.  Solve algebraically.
  //
  // 2.0 * (tb - (1 + d.ma) * ta - d.ba) + 2.0 * ((1.0 + d.mb) * tb - ta +
  // d.bb) * (1.0 + d.mb) = 0;
  //
  // tb - (1 + d.ma) * ta - d.ba + ((1.0 + d.mb) *
  // tb - ta + d.bb) * (1.0 + d.mb) = 0;
  //
  // tb - (1 + d.ma) * ta - d.ba + (1 + d.mb) (1 + d.mb) * tb - (1 + d.mb) ta
  // + (1 + d.mb) d.bb = 0;
  //
  // (1 + (1 + d.mb) (1 + d.mb)) tb - ((1 + d.ma) + (1
  // + d.mb)) * ta - d.ba + (1 + d.mb) d.bb = 0;
  //
  // tb = (((1 + d.ma) + (1 + d.mb)) * ta + d.ba - (1 + d.mb) d.bb) / (1 + (1
  // + d.mb) (1 + d.mb))

  mpq_class mpq_ta(message_bridge::FromInt64(ta.time_since_epoch().count()));
  mpq_class one(1);
  mpq_class mpq_tb =
      (((one + ma) + (one + mb)) * mpq_ta + ba - (one + mb) * bb) /
      (one + (one + mb) * (one + mb));
  mpq_tb.canonicalize();
  return mpq_tb;
}

// Tests that an infinite precision solution matches our numeric solver solution
// for a couple of simple problems.
TEST(TimestampProblemTest, Solve) {
  const monotonic_clock::time_point e = monotonic_clock::epoch();
  const monotonic_clock::time_point ta = e + chrono::milliseconds(500);

  NoncausalTimestampFilter a;
  // Delivered at e, sent at e + offset.
  // Sent at 1.001, received at 0
  a.Sample(e, chrono::milliseconds(1001));
  a.Sample(e + chrono::milliseconds(1000), chrono::milliseconds(1001));
  a.Sample(e + chrono::milliseconds(3000), chrono::milliseconds(999));

  NoncausalTimestampFilter b;
  // Sent at 0.001s, received at 1.000s
  b.Sample(e + chrono::milliseconds(1000), -chrono::milliseconds(999));
  b.Sample(e + chrono::milliseconds(2000), -chrono::milliseconds(1000));
  b.Sample(e + chrono::milliseconds(4000), -chrono::milliseconds(1002));

  TimestampProblem problem(2);
  problem.set_base_clock(0, ta);
  problem.set_base_clock(1, e);
  problem.set_solution_node(0);
  problem.add_filter(0, &a, 1);
  problem.add_filter(1, &b, 0);

  // Solve the problem with infinate precision as a verification and compare the
  // result.
  {
    const std::vector<double> result = problem.Solve();

    mpq_class tb_mpq =
        SolveExact(a.FitLine(), b.FitLine(), problem.base_clock(0));
    EXPECT_EQ(tb_mpq.get_d(), result[0])
        << std::setprecision(12) << std::fixed << " Expected " << tb_mpq.get_d()
        << " " << tb_mpq << " got " << result[0];
  }

  // Solve some other timestamps for grins.
  {
    problem.set_base_clock(0, e + chrono::milliseconds(500));
    std::vector<double> result = problem.Solve();

    mpq_class tb_mpq =
        SolveExact(a.FitLine(), b.FitLine(), problem.base_clock(0));

    EXPECT_EQ(tb_mpq.get_d(), result[0])
        << std::setprecision(12) << std::fixed << " Expected " << tb_mpq.get_d()
        << " " << tb_mpq << " got " << result[0];
  }

  // Now do the second line segment.
  {
    NoncausalTimestampFilter a;
    a.Sample(e + chrono::milliseconds(1000), chrono::milliseconds(1001));
    a.Sample(e + chrono::milliseconds(3000), chrono::milliseconds(999));

    NoncausalTimestampFilter b;
    b.Sample(e + chrono::milliseconds(2000), -chrono::milliseconds(1000));
    b.Sample(e + chrono::milliseconds(4000), -chrono::milliseconds(1002));
    {
      problem.set_base_clock(0, e + chrono::milliseconds(1500));
      const std::vector<double> result = problem.Solve();

      mpq_class tb_mpq =
          SolveExact(a.FitLine(), b.FitLine(), problem.base_clock(0));

      EXPECT_NEAR(tb_mpq.get_d(), result[0], 1e-6)
          << std::setprecision(12) << std::fixed << " Expected "
          << tb_mpq.get_d() << " " << tb_mpq << " got " << result[0];
    }

    {
      problem.set_base_clock(0, e + chrono::milliseconds(1600));
      const std::vector<double> result = problem.Solve();

      mpq_class tb_mpq =
          SolveExact(a.FitLine(), b.FitLine(), problem.base_clock(0));

      EXPECT_EQ(tb_mpq.get_d(), result[0])
          << std::setprecision(12) << std::fixed << " Expected "
          << tb_mpq.get_d() << " " << tb_mpq << " got " << result[0];
    }
  }
}

// Tests that a single timestamp InterpolatedTimeConverter returns equal
// results.  1 second should be 1 second everywhere.
TEST(InterpolatedTimeConverterTest, OneTime) {
  const distributed_clock::time_point de = distributed_clock::epoch();
  const monotonic_clock::time_point me = monotonic_clock::epoch();

  TestingTimeConverter time_converter(3u);
  time_converter.AddNextTimestamp(
      de + chrono::seconds(0),
      {me + chrono::seconds(1), me + chrono::seconds(10),
       me + chrono::seconds(1000)});

  EXPECT_EQ(time_converter.FromDistributedClock(0, de - chrono::seconds(1)),
            me + chrono::seconds(0));
  EXPECT_EQ(time_converter.FromDistributedClock(1, de - chrono::seconds(1)),
            me + chrono::seconds(9));
  EXPECT_EQ(time_converter.FromDistributedClock(2, de - chrono::seconds(1)),
            me + chrono::seconds(999));
  EXPECT_EQ(time_converter.ToDistributedClock(0, me + chrono::seconds(0)),
            de - chrono::seconds(1));
  EXPECT_EQ(time_converter.ToDistributedClock(1, me + chrono::seconds(9)),
            de - chrono::seconds(1));
  EXPECT_EQ(time_converter.ToDistributedClock(2, me + chrono::seconds(999)),
            de - chrono::seconds(1));

  EXPECT_EQ(time_converter.FromDistributedClock(0, de),
            me + chrono::seconds(1));
  EXPECT_EQ(time_converter.FromDistributedClock(1, de),
            me + chrono::seconds(10));
  EXPECT_EQ(time_converter.FromDistributedClock(2, de),
            me + chrono::seconds(1000));
  EXPECT_EQ(time_converter.ToDistributedClock(0, me + chrono::seconds(1)), de);
  EXPECT_EQ(time_converter.ToDistributedClock(1, me + chrono::seconds(10)), de);
  EXPECT_EQ(time_converter.ToDistributedClock(2, me + chrono::seconds(1000)),
            de);
}

// Tests that actual interpolation works as expected for multiple timestamps.
TEST(InterpolatedTimeConverterTest, Interpolation) {
  const distributed_clock::time_point de = distributed_clock::epoch();
  const monotonic_clock::time_point me = monotonic_clock::epoch();

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
      time_converter.FromDistributedClock(0, de + chrono::milliseconds(500)),
      me + chrono::milliseconds(1500));
  EXPECT_EQ(
      time_converter.FromDistributedClock(1, de + chrono::milliseconds(500)),
      me + chrono::milliseconds(10500));
  EXPECT_EQ(
      time_converter.FromDistributedClock(2, de + chrono::milliseconds(500)),
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

  EXPECT_EQ(
      time_converter.FromDistributedClock(0, de + chrono::milliseconds(2500)),
      me + chrono::milliseconds(3497));
  EXPECT_EQ(
      time_converter.FromDistributedClock(1, de + chrono::milliseconds(2500)),
      me + chrono::milliseconds(12498));
  EXPECT_EQ(
      time_converter.FromDistributedClock(2, de + chrono::milliseconds(2500)),
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

// Tests that reading times before the start of our interpolation points
// explodes.
TEST(InterpolatedTimeConverterDeathTest, ReadLostTime) {
  const distributed_clock::time_point de = distributed_clock::epoch();
  const monotonic_clock::time_point me = monotonic_clock::epoch();

  TestingTimeConverter time_converter(3u);
  time_converter.StartEqual();

  // Test that 2 timestamps interpolate correctly.
  for (int i = 0; i < 200; ++i) {
    time_converter.AddMonotonic({chrono::milliseconds(100),
                                 chrono::milliseconds(100),
                                 chrono::milliseconds(100)});
  }

  // Force 5 seconds to be read.
  EXPECT_EQ(
      de + chrono::milliseconds(5000),
      time_converter.ToDistributedClock(0, me + chrono::milliseconds(5000)));
  EXPECT_EQ(
      me + chrono::milliseconds(5000),
      time_converter.FromDistributedClock(0, de + chrono::milliseconds(5000)));

  // Double check we can read things from before the start
  EXPECT_EQ(
      de - chrono::milliseconds(100),
      time_converter.ToDistributedClock(0, me - chrono::milliseconds(100)));
  EXPECT_EQ(
      me - chrono::milliseconds(100),
      time_converter.FromDistributedClock(0, de - chrono::milliseconds(100)));

  // And at and after the origin.
  EXPECT_EQ(de, time_converter.ToDistributedClock(0, me));
  EXPECT_EQ(me, time_converter.FromDistributedClock(0, de));

  EXPECT_EQ(
      de + chrono::milliseconds(100),
      time_converter.ToDistributedClock(0, me + chrono::milliseconds(100)));
  EXPECT_EQ(
      me + chrono::milliseconds(100),
      time_converter.FromDistributedClock(0, de + chrono::milliseconds(100)));

  // Force 10.1 seconds now.  This will forget the 0th point at the origin.
  EXPECT_EQ(
      de + chrono::milliseconds(10100),
      time_converter.ToDistributedClock(0, me + chrono::milliseconds(10100)));
  EXPECT_EQ(
      me + chrono::milliseconds(10100),
      time_converter.FromDistributedClock(0, de + chrono::milliseconds(10100)));

  // Yup, can't read the origin anymore.
  EXPECT_DEATH({ LOG(INFO) << time_converter.ToDistributedClock(0, me); },
               "forgotten");
  EXPECT_DEATH({ LOG(INFO) << time_converter.FromDistributedClock(0, de); },
               "forgotten");

  // But can still read the next point.
  EXPECT_EQ(
      de + chrono::milliseconds(100),
      time_converter.ToDistributedClock(0, me + chrono::milliseconds(100)));
  EXPECT_EQ(
      me + chrono::milliseconds(100),
      time_converter.FromDistributedClock(0, de + chrono::milliseconds(100)));
}

// Tests unity time with 1 node.
TEST(InterpolatedTimeConverterTest, SingleNodeTime) {
  const distributed_clock::time_point de = distributed_clock::epoch();
  const monotonic_clock::time_point me = monotonic_clock::epoch();

  TestingTimeConverter time_converter(1u);
  time_converter.AddNextTimestamp(de + chrono::seconds(0),
                                  {me + chrono::seconds(1)});

  EXPECT_EQ(time_converter.FromDistributedClock(0, de), me);
  EXPECT_EQ(time_converter.FromDistributedClock(0, de + chrono::seconds(100)),
            me + chrono::seconds(100));

  EXPECT_TRUE(time_converter.NextTimestamp());
}

}  // namespace testing
}  // namespace message_bridge
}  // namespace aos
