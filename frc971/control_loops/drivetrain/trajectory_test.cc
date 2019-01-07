#include "frc971/control_loops/drivetrain/trajectory.h"

#include <chrono>
#include <vector>

#include "aos/testing/test_shm.h"
#include "gtest/gtest.h"
#include "y2016/constants.h"
#include "y2016/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2016/control_loops/drivetrain/hybrid_velocity_drivetrain.h"
#include "y2016/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"
#include "y2016/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

namespace chrono = ::std::chrono;

double a(double /*v*/, double /*x*/) { return 2.0; }

// Tests that the derivitives of xy integrate back up to the position.
TEST(IntegrateAccelForDistanceTest, IntegrateAccelForDistance) {
  double v = 0.0;
  const size_t count = 10;
  const double dx = 4.0 / static_cast<double>(count);
  for (size_t i = 0; i < count; ++i) {
    v = IntegrateAccelForDistance(a, v, 0.0, dx);
  }
  EXPECT_NEAR(4.0, v, 1e-8);
}

const constants::ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0, 0.25,
                                                           0.75};

// TODO(austin): factor this out of drivetrain_lib_test.cc
const DrivetrainConfig<double> &GetDrivetrainConfig() {
  static DrivetrainConfig<double> kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::HALL_EFFECT_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,
      ::frc971::control_loops::drivetrain::GyroType::SPARTAN_GYRO,
      IMUType::IMU_X,
      ::y2016::control_loops::drivetrain::MakeDrivetrainLoop,
      ::y2016::control_loops::drivetrain::MakeVelocityDrivetrainLoop,
      ::y2016::control_loops::drivetrain::MakeKFDrivetrainLoop,
      ::y2016::control_loops::drivetrain::MakeHybridVelocityDrivetrainLoop,

      chrono::duration_cast<chrono::nanoseconds>(
          chrono::duration<double>(::y2016::control_loops::drivetrain::kDt)),
      ::y2016::control_loops::drivetrain::kRobotRadius,
      ::y2016::control_loops::drivetrain::kWheelRadius,
      ::y2016::control_loops::drivetrain::kV,

      ::y2016::control_loops::drivetrain::kHighGearRatio,
      ::y2016::control_loops::drivetrain::kLowGearRatio,
      kThreeStateDriveShifter,
      kThreeStateDriveShifter,
      false,
      0,

      0.25,
      1.00,
      1.00};

  return kDrivetrainConfig;
};

class SplineTest : public ::testing::Test {
 protected:
  ::aos::testing::TestSharedMemory shm_;
  static constexpr chrono::nanoseconds kDt =
      chrono::duration_cast<chrono::nanoseconds>(
          chrono::duration<double>(::y2016::control_loops::drivetrain::kDt));

  DistanceSpline distance_spline_;
  Trajectory trajectory_;
  ::std::vector<::Eigen::Matrix<double, 3, 1>> length_plan_xva_;

  SplineTest()
      : distance_spline_(Spline((::Eigen::Matrix<double, 2, 4>() << 0.0, 1.2,
                                 -0.2, 1.0, 0.0, 0.0, 1.0, 1.0)
                                    .finished())),
        trajectory_(&distance_spline_, GetDrivetrainConfig()) {
    trajectory_.set_lateral_acceleration(2.0);
    trajectory_.set_longitudal_acceleration(1.0);
    trajectory_.LateralAccelPass();
    trajectory_.ForwardPass();
    trajectory_.BackwardPass();
    length_plan_xva_ = trajectory_.PlanXVA(kDt);
  }

  static constexpr double kXPos = 0.05;
  static constexpr double kYPos = 0.05;
  static constexpr double kThetaPos = 0.2;
  static constexpr double kVel = 0.5;
  const ::Eigen::DiagonalMatrix<double, 5> Q =
      (::Eigen::DiagonalMatrix<double, 5>().diagonal()
           << 1.0 / ::std::pow(kXPos, 2),
       1.0 / ::std::pow(kYPos, 2), 1.0 / ::std::pow(kThetaPos, 2),
       1.0 / ::std::pow(kVel, 2), 1.0 / ::std::pow(kVel, 2))
          .finished()
          .asDiagonal();

  const ::Eigen::DiagonalMatrix<double, 2> R =
      (::Eigen::DiagonalMatrix<double, 2>().diagonal()
           << 1.0 / ::std::pow(12.0, 2),
       1.0 / ::std::pow(12.0, 2))
          .finished()
          .asDiagonal();
};

constexpr chrono::nanoseconds SplineTest::kDt;

// Tests that following a spline with feed forwards only gets pretty darn close
// to the right point.
TEST_F(SplineTest, FFSpline) {
  ::Eigen::Matrix<double, 5, 1> state = ::Eigen::Matrix<double, 5, 1>::Zero();

  for (size_t i = 0; i < length_plan_xva_.size(); ++i) {
    const double distance = length_plan_xva_[i](0);

    const ::Eigen::Matrix<double, 2, 1> U_ff = trajectory_.FFVoltage(distance);
    const ::Eigen::Matrix<double, 2, 1> U = U_ff;
    state = RungeKuttaU(
        [this](const ::Eigen::Matrix<double, 5, 1> &X,
               const ::Eigen::Matrix<double, 2, 1> &U) {
          return ContinuousDynamics(trajectory_.velocity_drivetrain().plant(),
                                    trajectory_.Tlr_to_la(), X, U);
        },
        state, U, ::y2016::control_loops::drivetrain::kDt);
  }

  EXPECT_LT((state - trajectory_.GoalState(trajectory_.length(), 0.0)).norm(),
            2e-2);
}

// Tests that following a spline with both feed forwards and feed back gets
// pretty darn close to the right point.
TEST_F(SplineTest, FBSpline) {
  ::Eigen::Matrix<double, 5, 1> state = ::Eigen::Matrix<double, 5, 1>::Zero();

  for (size_t i = 0; i < length_plan_xva_.size(); ++i) {
    const double distance = length_plan_xva_[i](0);
    const double velocity = length_plan_xva_[i](1);
    const ::Eigen::Matrix<double, 5, 1> goal_state =
        trajectory_.GoalState(distance, velocity);

    const ::Eigen::Matrix<double, 5, 1> state_error = goal_state - state;

    const ::Eigen::Matrix<double, 2, 5> K =
        trajectory_.KForState(state, SplineTest::kDt, Q, R);

    const ::Eigen::Matrix<double, 2, 1> U_ff = trajectory_.FFVoltage(distance);
    const ::Eigen::Matrix<double, 2, 1> U_fb = K * state_error;
    const ::Eigen::Matrix<double, 2, 1> U = U_ff + U_fb;
    state = RungeKuttaU(
        [this](const ::Eigen::Matrix<double, 5, 1> &X,
               const ::Eigen::Matrix<double, 2, 1> &U) {
          return ContinuousDynamics(trajectory_.velocity_drivetrain().plant(),
                                    trajectory_.Tlr_to_la(), X, U);
        },
        state, U, ::y2016::control_loops::drivetrain::kDt);
  }

  EXPECT_LT((state - trajectory_.GoalState(trajectory_.length(), 0.0)).norm(),
            1.1e-2);
}

// TODO(austin): Handle saturation.  254 does this by just not going that
// fast...  We want to maybe replan when we get behind, or something.  Maybe
// stop moving the setpoint like our 2018 arm?

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
