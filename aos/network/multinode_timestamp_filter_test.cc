#include "aos/network/timestamp_filter.h"

#include <chrono>

#include <nlopt.h>
#include "aos/configuration.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/macros.h"
#include "aos/network/multinode_timestamp_filter.h"
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

}  // namespace testing
}  // namespace message_bridge
}  // namespace aos
