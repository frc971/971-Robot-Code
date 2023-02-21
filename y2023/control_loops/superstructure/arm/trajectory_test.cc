#include "y2023/control_loops/superstructure/arm/trajectory.h"

#include "frc971/control_loops/double_jointed_arm/dynamics.h"
#include "frc971/control_loops/double_jointed_arm/ekf.h"
#include "gtest/gtest.h"
#include "y2023/control_loops/superstructure/arm/arm_constants.h"
#include "y2023/control_loops/superstructure/roll/integral_hybrid_roll_plant.h"
#include "y2023/control_loops/superstructure/roll/integral_roll_plant.h"

namespace y2023 {
namespace control_loops {
namespace superstructure {
namespace arm {
namespace testing {

using frc971::control_loops::MatrixGaussianQuadrature5;
using frc971::control_loops::arm::Dynamics;
using frc971::control_loops::arm::EKF;

// Tests that we can pull out values along the path.
TEST(TrajectoryTest, Theta) {
  Eigen::Matrix<double, 2, 4> spline_params;
  spline_params << 0.3, 0.4, 0.6, 0.5, -1.7, -1.6, -1.4, -1.2;

  NSpline<4, 2> spline(spline_params);
  CosSpline cos_spline(spline,
                       {{0.0, 0.1}, {0.3, 0.1}, {0.7, 0.2}, {1.0, 0.2}});

  EXPECT_TRUE(cos_spline.Theta(-1.0).isApprox(
      (Eigen::Matrix<double, 3, 1>() << 0.3, -1.7, 0.1).finished()))
      << cos_spline.Theta(-1.0).transpose();

  EXPECT_TRUE(cos_spline.Theta(0.0).isApprox(
      (Eigen::Matrix<double, 3, 1>() << 0.3, -1.7, 0.1).finished()))
      << cos_spline.Theta(0.0).transpose();

  EXPECT_TRUE(cos_spline.Theta(0.3).isApprox(
      (Eigen::Matrix<double, 3, 1>() << 0.4062, -1.5857, 0.1).finished()))
      << cos_spline.Theta(0.3).transpose();

  EXPECT_TRUE(cos_spline.Theta(0.5).isApprox(
      (Eigen::Matrix<double, 3, 1>() << 0.475, -1.4875, 0.15).finished()))
      << cos_spline.Theta(0.5).transpose();

  EXPECT_TRUE(cos_spline.Theta(0.7).isApprox(
      (Eigen::Matrix<double, 3, 1>() << 0.5198, -1.3773, 0.2).finished()))
      << cos_spline.Theta(0.7).transpose();

  EXPECT_TRUE(cos_spline.Theta(1.0).isApprox(
      (Eigen::Matrix<double, 3, 1>() << 0.5, -1.2, 0.2).finished()))
      << cos_spline.Theta(1.0).transpose();

  EXPECT_TRUE(cos_spline.Theta(2.0).isApprox(
      (Eigen::Matrix<double, 3, 1>() << 0.5, -1.2, 0.2).finished()))
      << cos_spline.Theta(2.0).transpose();
}

// Tests that the integral of alpha and omega matches the functions they were
// differentiated from on Path.
TEST(TrajectoryTest, IntegrateAccel) {
  Eigen::Matrix<double, 2, 4> spline_params;
  spline_params << 0.3, 0.4, 0.6, 0.5, -1.7, -1.6, -1.4, -1.2;
  NSpline<4, 2> spline(spline_params);
  CosSpline cos_spline(spline,
                       {{0.0, 0.1}, {0.3, 0.1}, {0.7, 0.2}, {1.0, 0.2}});
  Path distance_spline(cos_spline, 100);

  Eigen::Matrix<double, 3, 1> integrated_theta = distance_spline.Theta(0.0);
  Eigen::Matrix<double, 3, 1> integrated_omega = distance_spline.Omega(0.0);

  constexpr size_t kSlices = 1000;
  for (size_t i = 0; i < kSlices; ++i) {
    const double d = i * distance_spline.length() / kSlices;
    const double next_d = (i + 1) * distance_spline.length() / kSlices;

    integrated_theta += MatrixGaussianQuadrature5<3>(
        [&](double distance) { return distance_spline.Omega(distance); }, d,
        next_d);
    integrated_omega += MatrixGaussianQuadrature5<3>(
        [&](double distance) { return distance_spline.Alpha(distance); }, d,
        next_d);

    EXPECT_TRUE(integrated_theta.isApprox(distance_spline.Theta(next_d), 1e-3))
        << ": Got " << integrated_theta.transpose() << ", expected "
        << distance_spline.Theta(next_d).transpose();
    EXPECT_TRUE(integrated_omega.isApprox(distance_spline.Omega(next_d), 1e-3))
        << ": Got " << integrated_omega.transpose() << ", expected "
        << distance_spline.Omega(next_d).transpose();
  }
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

std::unique_ptr<Path> MakeDemoPath() {
  Eigen::Matrix<double, 2, 4> spline_params;
  spline_params << 0.3, 0.4, 0.6, 0.5, -1.7, -1.6, -1.4, -1.2;
  NSpline<4, 2> spline(spline_params);
  CosSpline cos_spline(spline,
                       {{0.0, 0.1}, {0.3, 0.1}, {0.7, 0.2}, {1.0, 0.2}});
  return std::make_unique<Path>(cos_spline, 100);
}

// Tests that we can correctly interpolate velocities between two points
TEST(TrajectoryTest, ReversedPath) {
  // Tests that a reversed path is actually reversed.
  ::std::unique_ptr<Path> path = MakeDemoPath();
  ::std::unique_ptr<Path> reversed_path = Path::Reversed(MakeDemoPath());

  EXPECT_NEAR(path->length(), reversed_path->length(), 1e-6);

  for (double d = 0; d < path->length(); d += 0.01) {
    EXPECT_LT(
        (path->Theta(d) - reversed_path->Theta(path->length() - d)).norm(),
        1e-5);
    EXPECT_LT(
        (path->Omega(d) + reversed_path->Omega(path->length() - d)).norm(),
        1e-5);
    EXPECT_LT(
        (path->Alpha(d) - reversed_path->Alpha(path->length() - d)).norm(),
        1e-5);
  }
}

// Tests that we can follow a path.  Look at :trajectory_plot if you want to see
// the path.
TEST(TrajectoryTest, RunTrajectory) {
  Dynamics dynamics(kArmConstants);
  StateFeedbackLoop<3, 1, 1, double, StateFeedbackHybridPlant<3, 1, 1>,
                    HybridKalman<3, 1, 1>>
      hybrid_roll = y2023::control_loops::superstructure::roll::
          MakeIntegralHybridRollLoop();
  ::std::unique_ptr<Path> path = MakeDemoPath();
  Trajectory trajectory(&dynamics, &hybrid_roll.plant(), ::std::move(path),
                        0.001);

  constexpr double kAlpha0Max = 40.0;
  constexpr double kAlpha1Max = 60.0;
  constexpr double kAlpha2Max = 60.0;
  constexpr double vmax = 11.95;

  const ::Eigen::DiagonalMatrix<double, 3> alpha_unitizer(
      (::Eigen::DiagonalMatrix<double, 3>().diagonal() << (1.0 / kAlpha0Max),
       (1.0 / kAlpha1Max), (1.0 / kAlpha2Max))
          .finished());
  trajectory.OptimizeTrajectory(alpha_unitizer, vmax);

  double t = 0;
  ::Eigen::Matrix<double, 4, 1> arm_X;
  ::Eigen::Matrix<double, 2, 1> roll_X;
  {
    ::Eigen::Matrix<double, 3, 1> theta_t = trajectory.ThetaT(0.0);
    arm_X << theta_t(0), 0.0, theta_t(1), 0.0;
    roll_X << theta_t(2), 0.0;
  }

  EKF arm_ekf(&dynamics);
  arm_ekf.Reset(arm_X);
  StateFeedbackLoop<3, 1, 1, double, StateFeedbackPlant<3, 1, 1>,
                    StateFeedbackObserver<3, 1, 1>>
      roll = y2023::control_loops::superstructure::roll::MakeIntegralRollLoop();
  roll.mutable_X_hat().setZero();
  roll.mutable_X_hat().block<2, 1>(0, 0) = roll_X;

  TrajectoryFollower follower(&dynamics, &hybrid_roll, &trajectory);
  constexpr double sim_dt = 0.00505;
  while (t < 1.0) {
    arm_ekf.Correct(
        (::Eigen::Matrix<double, 2, 1>() << arm_X(0), arm_X(2)).finished(),
        sim_dt);
    roll.Correct((::Eigen::Matrix<double, 1, 1>() << roll_X(0)).finished());
    follower.Update(
        (Eigen::Matrix<double, 9, 1>() << arm_ekf.X_hat(), roll.X_hat())
            .finished(),
        false, sim_dt, vmax, 12.0);

    arm_X = dynamics.UnboundedDiscreteDynamics(
        arm_X, follower.U().block<2, 1>(0, 0), sim_dt);
    arm_ekf.Predict(follower.U().block<2, 1>(0, 0), sim_dt);

    roll_X =
        roll.plant()
            .Update((Eigen::Matrix<double, 3, 1>() << roll_X, 0.0).finished(),
                    follower.U().block<1, 1>(2, 0))
            .block<2, 1>(0, 0);
    roll.UpdateObserver(follower.U().block<1, 1>(2, 0),
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::duration<double>(sim_dt)));
    t += sim_dt;
  }

  ::Eigen::Matrix<double, 4, 1> final_arm_X;
  ::Eigen::Matrix<double, 2, 1> final_roll_X;
  ::Eigen::Matrix<double, 3, 1> final_theta_t =
      trajectory.ThetaT(trajectory.path().length());
  final_arm_X << final_theta_t(0), 0.0, final_theta_t(1), 0.0;
  final_roll_X << final_theta_t(2), 0.0;

  // Verify that we got to the end.
  EXPECT_TRUE(arm_X.isApprox(final_arm_X, 0.01))
      << ": X is " << arm_X.transpose() << " final_X is "
      << final_arm_X.transpose();
  EXPECT_TRUE(roll_X.isApprox(final_roll_X, 0.01))
      << ": X is " << roll_X.transpose() << " final_X is "
      << final_roll_X.transpose();

  // Verify that our goal is at the end.
  EXPECT_TRUE(
      final_theta_t.isApprox(trajectory.path().Theta(follower.goal(0))));
}

}  // namespace testing
}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2023
