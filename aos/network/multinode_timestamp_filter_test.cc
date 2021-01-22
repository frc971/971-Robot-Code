#include "aos/network/timestamp_filter.h"

#include <chrono>

#include "aos/configuration.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/macros.h"
#include "aos/network/multinode_timestamp_filter.h"
#include "aos/network/testing_time_converter.h"
#include "gtest/gtest.h"
#include "nlopt.h"
#include "third_party/gmp/gmpxx.h"

namespace aos {
namespace message_bridge {
namespace testing {

namespace chrono = std::chrono;
using aos::monotonic_clock;

// Converts a int64_t into a mpq_class.  This only uses 32 bit precision
// internally, so it will work on ARM.  This should only be used on 64 bit
// platforms to test out the 32 bit implementation.
inline mpq_class FromInt64(int64_t i) {
  uint64_t absi = std::abs(i);
  mpq_class bits(static_cast<uint32_t>((absi >> 32) & 0xffffffffu));
  bits *= mpq_class(0x10000);
  bits *= mpq_class(0x10000);
  bits += mpq_class(static_cast<uint32_t>(absi & 0xffffffffu));

  if (i < 0) {
    return -bits;
  } else {
    return bits;
  }
}

// Class to hold an affine function for the time offset.
// O(t) = slope * t + offset
//
// This is stored using mpq_class, which stores everything as full rational
// fractions.
class Line {
 public:
  Line() {}

  // Constructs a line given the offset and slope.
  Line(mpq_class offset, mpq_class slope) : offset_(offset), slope_(slope) {}

  // TODO(austin): Remove this one.
  Line(std::chrono::nanoseconds offset, double slope)
      : offset_(DoFromInt64(offset.count())), slope_(slope) {}

  // Fits a line to 2 points and returns the associated line.
  static Line Fit(
      const std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds> a,
      const std::tuple<monotonic_clock::time_point, std::chrono::nanoseconds>
          b);

  // Returns the full precision slopes and offsets.
  mpq_class mpq_offset() const { return offset_; }
  mpq_class mpq_slope() const { return slope_; }
  void increment_mpq_offset(mpq_class increment) { offset_ += increment; }

  // Returns the rounded offsets and slopes.
  std::chrono::nanoseconds offset() const {
    double o = offset_.get_d();
    return std::chrono::nanoseconds(static_cast<int64_t>(o));
  }
  double slope() const { return slope_.get_d(); }

  std::string DebugString() const {
    std::stringstream ss;
    ss << "Offset " << mpq_offset() << " slope " << mpq_slope();
    return ss.str();
  }

  void Debug() const {
    LOG(INFO) << DebugString();
  }

  // Returns the offset at a given time.
  // TODO(austin): get_d() ie double -> int64 can't be accurate...
  std::chrono::nanoseconds Eval(monotonic_clock::time_point pt) const {
    mpq_class result =
        mpq_class(FromInt64(pt.time_since_epoch().count())) * slope_ + offset_;
    return std::chrono::nanoseconds(static_cast<int64_t>(result.get_d()));
  }

 private:
  static mpq_class DoFromInt64(int64_t i) {
#if GMP_NUMB_BITS == 32
    return FromInt64(i);
#else
    return i;
#endif
  }

  mpq_class offset_;
  mpq_class slope_;
};

Line Line::Fit(
    const std::tuple<monotonic_clock::time_point, chrono::nanoseconds> a,
    const std::tuple<monotonic_clock::time_point, chrono::nanoseconds> b) {
  mpq_class slope = FromInt64((std::get<1>(b) - std::get<1>(a)).count()) /
                    FromInt64((std::get<0>(b) - std::get<0>(a)).count());
  slope.canonicalize();
  mpq_class offset =
      FromInt64(std::get<1>(a).count()) -
      FromInt64(std::get<0>(a).time_since_epoch().count()) * slope;
  offset.canonicalize();
  Line f(offset, slope);
  return f;
}

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

  mpq_class mpq_ta(FromInt64(ta.time_since_epoch().count()));
  mpq_class one(1);
  mpq_class mpq_tb =
      (((one + ma) + (one + mb)) * mpq_ta + ba - (one + mb) * bb) /
      (one + (one + mb) * (one + mb));
  mpq_tb.canonicalize();
  return mpq_tb;
}

Line FitLine(const NoncausalTimestampFilter &filter) {
  if (filter.timestamps_size() == 1) {
    Line fit(std::get<1>(filter.timestamp(0)), 0.0);
    return fit;
  } else {
    return Line::Fit(filter.timestamp(0), filter.timestamp(1));
  }
}

// Tests that an infinite precision solution matches our numeric solver solution
// for a couple of simple problems.
TEST(TimestampProblemTest, Solve) {
  const monotonic_clock::time_point e = monotonic_clock::epoch();
  const monotonic_clock::time_point ta = e + chrono::milliseconds(500);

  NoncausalTimestampFilter a(nullptr, nullptr);
  // Delivered at e, sent at e + offset.
  // Sent at 1.001, received at 0
  a.Sample(e, chrono::milliseconds(1001));
  a.Sample(e + chrono::milliseconds(1000), chrono::milliseconds(1001));
  a.Sample(e + chrono::milliseconds(3000), chrono::milliseconds(999));

  NoncausalTimestampFilter b(nullptr, nullptr);
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
    const std::vector<double> result = problem.SolveDouble();

    mpq_class tb_mpq =
        SolveExact(FitLine(a), FitLine(b), problem.base_clock(0));
    EXPECT_EQ(tb_mpq.get_d(), result[0])
        << std::setprecision(12) << std::fixed << " Expected " << tb_mpq.get_d()
        << " " << tb_mpq << " got " << result[0];
  }

  // Solve some other timestamps for grins.
  {
    problem.set_base_clock(0, e + chrono::milliseconds(500));
    std::vector<double> result = problem.SolveDouble();

    mpq_class tb_mpq =
        SolveExact(FitLine(a), FitLine(b), problem.base_clock(0));

    EXPECT_EQ(tb_mpq.get_d(), result[0])
        << std::setprecision(12) << std::fixed << " Expected " << tb_mpq.get_d()
        << " " << tb_mpq << " got " << result[0];
  }

  // Now do the second line segment.
  {
    NoncausalTimestampFilter a(nullptr, nullptr);
    a.Sample(e + chrono::milliseconds(1000), chrono::milliseconds(1001));
    a.Sample(e + chrono::milliseconds(3000), chrono::milliseconds(999));

    NoncausalTimestampFilter b(nullptr, nullptr);
    b.Sample(e + chrono::milliseconds(2000), -chrono::milliseconds(1000));
    b.Sample(e + chrono::milliseconds(4000), -chrono::milliseconds(1002));
    {
      problem.set_base_clock(0, e + chrono::milliseconds(1500));
      const std::vector<double> result = problem.SolveDouble();

      mpq_class tb_mpq =
          SolveExact(FitLine(a), FitLine(b), problem.base_clock(0));

      EXPECT_NEAR(tb_mpq.get_d(), result[0], 1e-6)
          << std::setprecision(12) << std::fixed << " Expected "
          << tb_mpq.get_d() << " " << tb_mpq << " got " << result[0];
    }

    {
      problem.set_base_clock(0, e + chrono::milliseconds(1600));
      const std::vector<double> result = problem.SolveDouble();

      mpq_class tb_mpq =
          SolveExact(FitLine(a), FitLine(b), problem.base_clock(0));

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
  EXPECT_EQ(
      me + kDefaultHistoryDuration / 2,
      time_converter.FromDistributedClock(0, de + kDefaultHistoryDuration / 2));

  // Double check we can read things from before the start
  EXPECT_EQ(
      de - kDt,
      time_converter.ToDistributedClock(0, me - kDt));
  EXPECT_EQ(
      me - kDt,
      time_converter.FromDistributedClock(0, de - kDt));

  // And at and after the origin.
  EXPECT_EQ(de, time_converter.ToDistributedClock(0, me));
  EXPECT_EQ(me, time_converter.FromDistributedClock(0, de));

  EXPECT_EQ(
      de + chrono::milliseconds(10),
      time_converter.ToDistributedClock(0, me + kDt));
  EXPECT_EQ(
      me + chrono::milliseconds(10),
      time_converter.FromDistributedClock(0, de + kDt));

  // Force 10.1 seconds now.  This will forget the 0th point at the origin.
  EXPECT_EQ(
      de + kDefaultHistoryDuration + kDt,
      time_converter.ToDistributedClock(0, me + kDefaultHistoryDuration + kDt));
  EXPECT_EQ(
      me + kDefaultHistoryDuration + kDt,
      time_converter.FromDistributedClock(0, de + kDefaultHistoryDuration + kDt));

  // Yup, can't read the origin anymore.
  EXPECT_DEATH({ LOG(INFO) << time_converter.ToDistributedClock(0, me); },
               "forgotten");
  EXPECT_DEATH({ LOG(INFO) << time_converter.FromDistributedClock(0, de); },
               "forgotten");

  // But can still read the next point.
  EXPECT_EQ(
      de + kDt,
      time_converter.ToDistributedClock(0, me + kDt));
  EXPECT_EQ(
      me + kDt,
      time_converter.FromDistributedClock(0, de + kDt));
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
