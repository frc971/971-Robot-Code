#include "y2018/control_loops/superstructure/arm/trajectory.h"

#include "gtest/gtest.h"
#include "y2018/control_loops/superstructure/arm/demo_path.h"
#include "y2018/control_loops/superstructure/arm/dynamics.h"
#include "y2018/control_loops/superstructure/arm/ekf.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {
namespace testing {

// Tests that we can pull out values along the path.
TEST(TrajectoryTest, Theta) {
  Path p({{{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{1.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{2.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{3.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{4.0, 0.0, 1.0, 0.0, 0.0, 0.0}}});

  EXPECT_TRUE(p.Theta(0.5).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 0.5, 0.0).finished()));
  EXPECT_TRUE(p.Omega(0.5).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 1.0, 0.0).finished()));
  EXPECT_TRUE(p.Alpha(0.5).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 0.0, 0.0).finished()));

  EXPECT_TRUE(p.Theta(1.5).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 1.5, 0.0).finished()));

  for (double d = 0.0; d <= 4.0; d += 0.01) {
    EXPECT_TRUE(p.Theta(d).isApprox(
        (::Eigen::Matrix<double, 2, 1>() << d, 0.0).finished()));
  }
}

// Tests a path where picking the right point to interpolate for matters.
TEST(TrajectoryTest, HarderPath) {
  Path p({{{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{1.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{1.0, 1.0, 0.0, 1.0, 0.0, 0.0}}});

  EXPECT_TRUE(p.Theta(0.5).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 0.5, 0.0).finished()));

  EXPECT_TRUE(p.Theta(1.5).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 1.0, 0.5).finished()));
}

// Tests that a path with an even number of points works as expected.
TEST(TrajectoryTest, EvenPointNumbers) {
  Path p({{{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{1.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{2.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{3.0, 0.0, 1.0, 0.0, 0.0, 0.0}}});
  EXPECT_TRUE(p.Theta(1.5).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 1.5, 0.0).finished()));

  for (double d = 0.0; d <= 3.0; d += 0.01) {
    EXPECT_TRUE(p.Theta(d).isApprox(
        (::Eigen::Matrix<double, 2, 1>() << d, 0.0).finished()));
  }
}

// Tests that out of range points work as expected.
TEST(TrajectoryTest, OutOfBounds) {
  Path p({{{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{1.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{2.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{3.0, 0.0, 1.0, 0.0, 0.0, 0.0}}});
  EXPECT_TRUE(p.Theta(-1.0).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 0.0, 0.0).finished()));
  EXPECT_TRUE(p.Theta(10.0).isApprox(
      (::Eigen::Matrix<double, 2, 1>() << 3.0, 0.0).finished()));
}


// Tests that we can compute the indices of the plan for a given distance correctly.
TEST(TrajectoryTest, IndicesForDistanceTest) {
  // Start with a stupid simple plan.
  Path p({{{0.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{1.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{2.0, 0.0, 1.0, 0.0, 0.0, 0.0}},
          {{3.0, 0.0, 1.0, 0.0, 0.0, 0.0}}});
  Trajectory t(::std::unique_ptr<Path>(new Path(p)), 0.1);

  // 0 - 3.0 every 0.1 should be 31 points.
  EXPECT_EQ(t.num_plan_points(), 31);

  // Verify that something centered in a grid cell returns the points on either side.
  EXPECT_EQ(::std::make_pair(static_cast<size_t>(0), static_cast<size_t>(1)),
            t.IndicesForDistance(0.05));
  EXPECT_EQ(::std::make_pair(static_cast<size_t>(1), static_cast<size_t>(2)),
            t.IndicesForDistance(0.15));

  // Verify that something on an edge returns the expected result.
  EXPECT_EQ(::std::make_pair(static_cast<size_t>(1), static_cast<size_t>(2)),
            t.IndicesForDistance(0.1));

  // Verify what small deviations result.
  EXPECT_EQ(::std::make_pair(static_cast<size_t>(1), static_cast<size_t>(2)),
            t.IndicesForDistance(0.1001));
  EXPECT_EQ(::std::make_pair(static_cast<size_t>(0), static_cast<size_t>(1)),
            t.IndicesForDistance(0.0999));

  // Verify that blowing past the ends works.
  EXPECT_EQ(::std::make_pair(static_cast<size_t>(29), static_cast<size_t>(30)),
            t.IndicesForDistance(4.0));
  EXPECT_EQ(::std::make_pair(static_cast<size_t>(0), static_cast<size_t>(1)),
            t.IndicesForDistance(-0.1));

  // Verify that the index to distance calculation also works.
  EXPECT_EQ(0.0, t.DistanceForIndex(0));
  EXPECT_EQ(0.1, t.DistanceForIndex(1));
  EXPECT_EQ(3.0, t.DistanceForIndex(30));
}

// Tests that we can correctly interpolate velocities between two points
TEST(TrajectoryTest, InterpolateVelocity) {
  // x = 0.5 * a * t^2
  // v = a * t
  // a = 2.0
  EXPECT_EQ(0.0, Trajectory::InterpolateVelocity(0.0, 0.0, 1.0, 0.0, 2.0));
  EXPECT_EQ(2.0, Trajectory::InterpolateVelocity(1.0, 0.0, 1.0, 0.0, 2.0));
  EXPECT_EQ(0.0, Trajectory::InterpolateVelocity(-1.0, 0.0, 1.0, 0.0, 2.0));
  EXPECT_EQ(2.0, Trajectory::InterpolateVelocity(20.0, 0.0, 1.0, 0.0, 2.0));
}

// Tests that we can correctly interpolate velocities between two points
TEST(TrajectoryTest, InterpolateAcceleration) {
  // x = 0.5 * a * t^2
  // v = a * t
  // a = 2.0
  EXPECT_EQ(2.0, Trajectory::InterpolateAcceleration(0.0, 1.0, 0.0, 2.0));
}

// Tests that we can correctly interpolate velocities between two points
TEST(TrajectoryTest, ReversedPath) {
  // Tests that a reversed path is actually reversed.
  ::std::unique_ptr<Path> path = MakeDemoPath();
  ::std::unique_ptr<Path> reversed_path(
      new Path(Path::Reversed(*MakeDemoPath())));

  EXPECT_NEAR(path->length(), reversed_path->length(), 1e-6);

  for (double d = 0; d < path->length(); d += 0.01) {
    EXPECT_TRUE(path->Theta(d).isApprox(reversed_path->Theta(path->length() - d)));
    EXPECT_TRUE(path->Omega(d).isApprox(-reversed_path->Omega(path->length() - d)));
    EXPECT_TRUE(path->Alpha(d).isApprox(reversed_path->Alpha(path->length() - d)));
  }
}

// Tests that we can follow a path.  Look at :trajectory_plot if you want to see
// the path.
TEST(TrajectoryTest, RunTrajectory) {
  ::std::unique_ptr<Path> path = MakeDemoPath();
  Trajectory trajectory(::std::move(path), 0.001);

  constexpr double kAlpha0Max = 40.0;
  constexpr double kAlpha1Max = 60.0;
  constexpr double vmax = 11.95;

  const ::Eigen::Matrix<double, 2, 2> alpha_unitizer =
      (::Eigen::Matrix<double, 2, 2>() << 1.0 / kAlpha0Max, 0.0, 0.0,
       1.0 / kAlpha1Max)
          .finished();
  trajectory.OptimizeTrajectory(alpha_unitizer, vmax);

  double t = 0;
  ::Eigen::Matrix<double, 4, 1> X;
  {
    ::Eigen::Matrix<double, 2, 1> theta_t = trajectory.ThetaT(0.0);
    X << theta_t(0), 0.0, theta_t(1), 0.0;
  }

  EKF arm_ekf;
  arm_ekf.Reset(X);

  TrajectoryFollower follower(&trajectory);
  constexpr double sim_dt = 0.00505;
  while (t < 1.0) {
    arm_ekf.Correct((::Eigen::Matrix<double, 2, 1>() << X(0), X(2)).finished(),
                    sim_dt);
    follower.Update(arm_ekf.X_hat(), false, sim_dt, vmax, 12.0);
    X = Dynamics::UnboundedDiscreteDynamics(X, follower.U(), sim_dt);
    arm_ekf.Predict(follower.U(), sim_dt);
    t += sim_dt;
  }

  ::Eigen::Matrix<double, 4, 1> final_X;
  ::Eigen::Matrix<double, 2, 1> final_theta_t =
      trajectory.ThetaT(trajectory.path().length());
  final_X << final_theta_t(0), 0.0, final_theta_t(1), 0.0;

  // Verify that we got to the end.
  EXPECT_TRUE(X.isApprox(final_X, 0.01))
      << ": X is " << X.transpose() << " final_X is " << final_X.transpose();

  // Verify that our goal is at the end.
  EXPECT_TRUE(
      final_theta_t.isApprox(trajectory.path().Theta(follower.goal(0))));
}

}  // namespace testing
}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
