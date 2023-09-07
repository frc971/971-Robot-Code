#include "aos/network/multinode_timestamp_filter.h"

#include <chrono>

#include "gtest/gtest.h"

#include "aos/configuration.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/macros.h"
#include "aos/network/testing_time_converter.h"
#include "aos/network/timestamp_filter.h"

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

class SquareProblem : public Problem {
 public:
  SquareProblem() : Problem() {
    inequality_constraints_ = 4u;
    states_ = 2u;
  }

  void Prepare(size_t my_solve_number) override {
    LOG(INFO) << "Starting solve " << my_solve_number;
  }

  void Update(size_t /*solve_number*/,
              Eigen::Ref<Eigen::VectorXd> /*y*/) override {}

  Derivatives ComputeDerivatives(
      size_t /*solve_number*/, const Eigen::Ref<const Eigen::VectorXd> y,
      bool quiet, bool all,
      const absl::Span<const size_t> active_constraints) override {
    Eigen::Matrix<double, 2, 2> Q = Eigen::DiagonalMatrix<double, 2>(1.0, 2.0);
    Eigen::Matrix<double, 4, 2> H;

    // Box constraint from -1 <= x(0) <= 5, and 1 <= x(1) <= 5
    H << 1.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, -1.0;
    Eigen::Matrix<double, 4, 1> k;
    k << 5.0, 1.0, 5.0, -1.0;

    Derivatives result;
    result.solution_node = 0;
    result.hessian = 2.0 * Q;
    result.gradient = 2.0 * Q * y.block<2, 1>(0, 0);

    if (all || active_constraints.size() != 0u) {
      if (!all) {
        CHECK_EQ(active_constraints.size(), 4u);
      }
      result.f = H * y.block<2, 1>(0, 0) - k;
      result.df = H;
      result.df_slope_limited = result.df;
    }

    result.A = Eigen::MatrixXd::Zero(1, 2);
    // Set x0 = x1
    result.A(0, 0) = 1.0;
    result.A(0, 1) = -1.0;

    result.Axmb = result.A * y.block<2, 1>(0, 0);

    if (!quiet) {
      VLOG(2) << Q;
    }

    return result;
  }
};

// Tests that our solver can solve a toy problem with known answers.
TEST(TimestampProblemTest, SolveToyNewton) {
  SquareProblem problem;

  NewtonSolver solver;
  Eigen::VectorXd y;
  size_t solution_node;
  size_t iterations;
  std::tie(y, solution_node, iterations) = solver.SolveNewton(&problem, 20);

  EXPECT_NEAR(y(0), 0.0, 1e-6);
  EXPECT_NEAR(y(1), 0.0, 1e-6);

  std::vector<size_t> constraints;
  for (size_t i = 0; i < problem.inequality_constraints(); ++i) {
    constraints.emplace_back(i);
  }

  std::vector<size_t> used_constraints;
  std::tie(y, solution_node, iterations, used_constraints) =
      solver.SolveConstrainedNewton(&problem, 20, constraints).value();

  LOG(INFO) << y.transpose();

  EXPECT_EQ(constraints, used_constraints);
  EXPECT_NEAR(y(0), 1.0, 1e-3);
  EXPECT_NEAR(y(1), 1.0, 1e-3);
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

  // Set up a time problem with an interesting shape that isn't simple and
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

  constexpr size_t kMaxIterations = 200u;
  problem.set_points(points1);

  NewtonSolver solver1;
  const std::tuple<Eigen::VectorXd, size_t, size_t> result1 =
      solver1.SolveNewton(&problem, kMaxIterations);
  const std::vector<logger::BootTimestamp> times1 =
      problem.PackResults(false, std::get<0>(result1));
  EXPECT_LT(std::get<2>(result1), kMaxIterations);
  EXPECT_EQ(std::get<1>(result1), 0u);
  EXPECT_TRUE(problem.ValidateSolution(times1, false));

  std::vector<BootTimestamp> points2(problem.size(), BootTimestamp::max_time());
  points2[1] = times1[1];
  problem.set_points(points2);
  NewtonSolver solver2;
  std::tuple<Eigen::VectorXd, size_t, size_t> result2 =
      solver2.SolveNewton(&problem, kMaxIterations);
  const std::vector<logger::BootTimestamp> times2 =
      problem.PackResults(false, std::get<0>(result2));
  EXPECT_LT(std::get<2>(result1), kMaxIterations);
  EXPECT_EQ(std::get<1>(result2), 1u);
  EXPECT_TRUE(problem.ValidateSolution(times2, false));

  EXPECT_EQ(times1[0], e + chrono::seconds(1));
  EXPECT_EQ(times1[0], times2[0]);
  EXPECT_EQ(times1[1], times2[1]);

  // Confirm that the error is almost equal for both directions.  The solution
  // is an integer solution, so there will be a little bit of error left over.
  std::tuple<chrono::nanoseconds, double, double> a_error =
      a.OffsetError(nullptr, NoncausalTimestampFilter::Pointer(), times1[0],
                    0.0, times1[1], 0.0)
          .second;
  std::tuple<chrono::nanoseconds, double, double> b_error =
      b.OffsetError(nullptr, NoncausalTimestampFilter::Pointer(), times1[1],
                    0.0, times1[0], 0.0)
          .second;
  EXPECT_NEAR(static_cast<double>(
                  (std::get<0>(a_error) - std::get<0>(b_error)).count()) +
                  (std::get<1>(a_error) - std::get<1>(b_error)),
              0.0, 0.5);

  // Confirm the derivatives for the solution make sense.
  Problem::Derivatives derivatives = problem.ComputeDerivatives(
      0, std::get<0>(result2), true, true, absl::Span<const size_t>());

  // Hessian should be fixed and pretty simple.
  Eigen::Matrix<double, 2, 2> expected_hessian;
  expected_hessian << 4, -4, -4, 4;
  EXPECT_NEAR((derivatives.hessian - expected_hessian).norm(), 0.0, 1e-6)
      << derivatives.hessian;

  // It's the solution, gradient should be small.
  EXPECT_NEAR(derivatives.gradient.norm(), 0.0, 1e-6) << derivatives.gradient;

  Eigen::Matrix<double, 2, 1> expected_f;
  expected_f << -1500248.1249374687, -1500248.1249374687;
  EXPECT_NEAR((derivatives.f - expected_f).norm(), 0.0, 1e-6) << derivatives.f;

  // A -> 1/n for each element.
  Eigen::Matrix<double, 1, 2> expected_A;
  expected_A << 0.5, 0.5;
  EXPECT_NEAR((derivatives.A - expected_A).norm(), 0.0, 1e-12) << derivatives.A;

  // Ax - b -> 0 at the solution.
  EXPECT_NEAR(derivatives.Axmb.norm(), 0.0, 1e-4) << derivatives.Axmb;

  // And df is well defined.
  Eigen::Matrix<double, 2, 2> expected_df;
  expected_df << (1.0 - kMaxVelocity()), -1, -1, (1 - kMaxVelocity());
  EXPECT_NEAR((derivatives.df - expected_df).norm(), 0.0, 1e-12)
      << derivatives.df;

  Eigen::Matrix<double, 2, 2> expected_df_slope_limited;
  expected_df_slope_limited << 1.0 + kMaxVelocity(), -1, -1, 1 + kMaxVelocity();
  EXPECT_NEAR((derivatives.df_slope_limited - expected_df_slope_limited).norm(),
              0.0, 1e-12)
      << derivatives.df_slope_limited;
}

}  // namespace testing
}  // namespace message_bridge
}  // namespace aos
