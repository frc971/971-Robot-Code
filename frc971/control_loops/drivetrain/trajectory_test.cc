#include "frc971/control_loops/drivetrain/trajectory.h"

#include <chrono>
#include <vector>

#include "aos/testing/test_shm.h"
#include "gflags/gflags.h"
#include "gtest/gtest.h"
#if defined(SUPPORT_PLOT)
#include "third_party/matplotlib-cpp/matplotlibcpp.h"
#endif
#include "y2016/constants.h"
#include "y2016/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2016/control_loops/drivetrain/hybrid_velocity_drivetrain.h"
#include "y2016/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"
#include "y2016/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

DEFINE_bool(plot, false, "If true, plot");

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
      ::y2016::control_loops::drivetrain::kJ,
      ::y2016::control_loops::drivetrain::kMass,
      kThreeStateDriveShifter,
      kThreeStateDriveShifter,
      false,
      0,

      0.25,
      1.00,
      1.00};

  return kDrivetrainConfig;
};

struct SplineTestParams {
  ::Eigen::Matrix<double, 2, 4> control_points;
  double lateral_acceleration;
  double longitudal_acceleration;
  double velocity_limit;
  double voltage_limit;
  ::std::function<void(Trajectory *)> trajectory_modification_fn;
};

void NullTrajectoryModificationFunction(Trajectory *) {}

class ParameterizedSplineTest
    : public ::testing::TestWithParam<SplineTestParams> {
 public:
  ::aos::testing::TestSharedMemory shm_;
  static constexpr chrono::nanoseconds kDt =
      chrono::duration_cast<chrono::nanoseconds>(
          chrono::duration<double>(::y2016::control_loops::drivetrain::kDt));

  ::std::unique_ptr<DistanceSpline> distance_spline_;
  ::std::unique_ptr<Trajectory> trajectory_;
  ::std::vector<::Eigen::Matrix<double, 3, 1>> length_plan_xva_;

  ParameterizedSplineTest() {}

  void SetUp() {
    distance_spline_ = ::std::unique_ptr<DistanceSpline>(
        new DistanceSpline(Spline(Spline4To6(GetParam().control_points))));
    trajectory_ = ::std::unique_ptr<Trajectory>(
        new Trajectory(distance_spline_.get(), GetDrivetrainConfig(),
                       GetParam().velocity_limit));
    trajectory_->set_lateral_acceleration(GetParam().lateral_acceleration);
    trajectory_->set_longitudal_acceleration(GetParam().longitudal_acceleration);
    trajectory_->set_longitudal_acceleration(GetParam().longitudal_acceleration);
    trajectory_->set_voltage_limit(GetParam().voltage_limit);

    GetParam().trajectory_modification_fn(trajectory_.get());

    initial_plan_ = trajectory_->plan();
    trajectory_->LateralAccelPass();
    curvature_plan_ = trajectory_->plan();
    trajectory_->ForwardPass();
    forward_plan_ = trajectory_->plan();
    trajectory_->BackwardPass();
    backward_plan_ = trajectory_->plan();

    length_plan_xva_ = trajectory_->PlanXVA(kDt);
  }

  void TearDown() {
    printf("  Spline takes %f seconds to follow\n",
           length_plan_xva_.size() * ::y2016::control_loops::drivetrain::kDt);
#if defined(SUPPORT_PLOT)
    if (FLAGS_plot) {
      ::std::vector<double> distances = trajectory_->Distances();

      for (size_t i = 0; i < length_plan_xva_.size(); ++i) {
        length_plan_t_.push_back(i * ::y2016::control_loops::drivetrain::kDt);
        length_plan_x_.push_back(length_plan_xva_[i](0));
        length_plan_v_.push_back(length_plan_xva_[i](1));
        length_plan_a_.push_back(length_plan_xva_[i](2));
      }

      ::std::vector<double> plan_segment_center_distance;
      ::std::vector<double> plan_type;
      for (Trajectory::SegmentType segment_type :
           trajectory_->plan_segment_type()) {
        plan_type.push_back(static_cast<int>(segment_type));
      }
      for (size_t i = 0; i < distances.size() - 1; ++i) {
        plan_segment_center_distance.push_back(
            (distances[i] + distances[i + 1]) / 2.0);
      }

      matplotlibcpp::figure();
      matplotlibcpp::plot(plan_segment_center_distance, plan_type,
                          {{"label", "plan_type"}});
      matplotlibcpp::plot(distances, backward_plan_, {{"label", "backward"}});
      matplotlibcpp::plot(distances, forward_plan_, {{"label", "forward"}});
      matplotlibcpp::plot(distances, curvature_plan_, {{"label", "lateral"}});
      matplotlibcpp::plot(distances, initial_plan_, {{"label", "initial"}});
      matplotlibcpp::legend();

      matplotlibcpp::figure();
      matplotlibcpp::plot(length_plan_t_, length_plan_x_, {{"label", "x"}});
      matplotlibcpp::plot(length_plan_t_, length_plan_v_, {{"label", "v"}});
      matplotlibcpp::plot(length_plan_t_, length_plan_a_, {{"label", "a"}});
      matplotlibcpp::plot(length_plan_t_, length_plan_vl_, {{"label", "Vl"}});
      matplotlibcpp::plot(length_plan_t_, length_plan_vr_, {{"label", "Vr"}});
      matplotlibcpp::legend();

      matplotlibcpp::show();
    }
#endif
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

  ::std::vector<double> initial_plan_;
  ::std::vector<double> curvature_plan_;
  ::std::vector<double> forward_plan_;
  ::std::vector<double> backward_plan_;

  ::std::vector<double> length_plan_t_;
  ::std::vector<double> length_plan_x_;
  ::std::vector<double> length_plan_v_;
  ::std::vector<double> length_plan_a_;
  ::std::vector<double> length_plan_vl_;
  ::std::vector<double> length_plan_vr_;

};

constexpr chrono::nanoseconds ParameterizedSplineTest::kDt;

// Tests that following a spline with feed forwards only gets pretty darn close
// to the right point.
TEST_P(ParameterizedSplineTest, FFSpline) {
  ::Eigen::Matrix<double, 5, 1> state = ::Eigen::Matrix<double, 5, 1>::Zero();

  for (size_t i = 0; i < length_plan_xva_.size(); ++i) {
    const double distance = length_plan_xva_[i](0);

    const ::Eigen::Matrix<double, 2, 1> U_ff = trajectory_->FFVoltage(distance);
    const ::Eigen::Matrix<double, 2, 1> U = U_ff;

    length_plan_vl_.push_back(U(0));
    length_plan_vr_.push_back(U(1));
    state = RungeKuttaU(
        [this](const ::Eigen::Matrix<double, 5, 1> &X,
               const ::Eigen::Matrix<double, 2, 1> &U) {
          return ContinuousDynamics(trajectory_->velocity_drivetrain().plant(),
                                    trajectory_->Tlr_to_la(), X, U);
        },
        state, U, ::y2016::control_loops::drivetrain::kDt);
  }

  EXPECT_LT((state - trajectory_->GoalState(trajectory_->length(), 0.0)).norm(),
            2e-2);
}

// Tests that following a spline with both feed forwards and feed back gets
// pretty darn close to the right point.
TEST_P(ParameterizedSplineTest, FBSpline) {
  ::Eigen::Matrix<double, 5, 1> state = ::Eigen::Matrix<double, 5, 1>::Zero();

  for (size_t i = 0; i < length_plan_xva_.size(); ++i) {
    const double distance = length_plan_xva_[i](0);
    const double velocity = length_plan_xva_[i](1);
    const ::Eigen::Matrix<double, 5, 1> goal_state =
        trajectory_->GoalState(distance, velocity);

    const ::Eigen::Matrix<double, 5, 1> state_error = goal_state - state;

    const ::Eigen::Matrix<double, 2, 5> K =
        trajectory_->KForState(state, ParameterizedSplineTest::kDt, Q, R);

    const ::Eigen::Matrix<double, 2, 1> U_ff = trajectory_->FFVoltage(distance);
    const ::Eigen::Matrix<double, 2, 1> U_fb = K * state_error;
    const ::Eigen::Matrix<double, 2, 1> U = U_ff + U_fb;

    length_plan_vl_.push_back(U(0));
    length_plan_vr_.push_back(U(1));
    state = RungeKuttaU(
        [this](const ::Eigen::Matrix<double, 5, 1> &X,
               const ::Eigen::Matrix<double, 2, 1> &U) {
          return ContinuousDynamics(trajectory_->velocity_drivetrain().plant(),
                                    trajectory_->Tlr_to_la(), X, U);
        },
        state, U, ::y2016::control_loops::drivetrain::kDt);
  }

  EXPECT_LT((state - trajectory_->GoalState(trajectory_->length(), 0.0)).norm(),
            1.2e-2);
}

SplineTestParams MakeSplineTestParams(struct SplineTestParams params) {
  return params;
}

void LimitMiddleOfPathTrajectoryModificationFunction(Trajectory *trajectory) {
  trajectory->LimitVelocity(1.0, 2.0, 0.5);
}

void ShortLimitMiddleOfPathTrajectoryModificationFunction(Trajectory *trajectory) {
  trajectory->LimitVelocity(1.5, 1.5, 0.5);
}

INSTANTIATE_TEST_CASE_P(
    SplineTest, ParameterizedSplineTest,
    ::testing::Values(
        // Normal spline.
        MakeSplineTestParams({(::Eigen::Matrix<double, 2, 4>() << 0.0, 1.2,
                               -0.2, 1.0, 0.0, 0.0, 1.0, 1.0)
                                  .finished(),
                              2.0 /*lateral acceleration*/,
                              1.0 /*longitudinal acceleration*/,
                              10.0 /* velocity limit */, 12.0 /* volts */,
                              NullTrajectoryModificationFunction}),
        // Be velocity limited.
        MakeSplineTestParams({(::Eigen::Matrix<double, 2, 4>() << 0.0, 6.0,
                               -1.0, 5.0, 0.0, 0.0, 1.0, 1.0)
                                  .finished(),
                              2.0 /*lateral acceleration*/,
                              1.0 /*longitudinal acceleration*/,
                              0.5 /* velocity limit */, 12.0 /* volts */,
                              NullTrajectoryModificationFunction}),
        // Hit the voltage limit.
        MakeSplineTestParams({(::Eigen::Matrix<double, 2, 4>() << 0.0, 6.0,
                               -1.0, 5.0, 0.0, 0.0, 1.0, 1.0)
                                  .finished(),
                              2.0 /*lateral acceleration*/,
                              3.0 /*longitudinal acceleration*/,
                              10.0 /* velocity limit */, 5.0 /* volts */,
                              NullTrajectoryModificationFunction}),
        // Hit the curvature limit.
        MakeSplineTestParams({(::Eigen::Matrix<double, 2, 4>() << 0.0, 1.2,
                               -0.2, 1.0, 0.0, 0.0, 1.0, 1.0)
                                  .finished(),
                              1.0 /*lateral acceleration*/,
                              3.0 /*longitudinal acceleration*/,
                              10.0 /* velocity limit */, 12.0 /* volts */,
                              NullTrajectoryModificationFunction}),
        // Add an artifical velocity limit in the middle.
        MakeSplineTestParams({(::Eigen::Matrix<double, 2, 4>() << 0.0, 6.0,
                               -1.0, 5.0, 0.0, 0.0, 1.0, 1.0)
                                  .finished(),
                              2.0 /*lateral acceleration*/,
                              3.0 /*longitudinal acceleration*/,
                              10.0 /* velocity limit */, 12.0 /* volts */,
                              LimitMiddleOfPathTrajectoryModificationFunction}),
        // Add a really short artifical velocity limit in the middle.
        MakeSplineTestParams({(::Eigen::Matrix<double, 2, 4>() << 0.0, 6.0,
                               -1.0, 5.0, 0.0, 0.0, 1.0, 1.0)
                                  .finished(),
                              2.0 /*lateral acceleration*/,
                              3.0 /*longitudinal acceleration*/,
                              10.0 /* velocity limit */, 12.0 /* volts */,
                              ShortLimitMiddleOfPathTrajectoryModificationFunction})));

// TODO(austin): Handle saturation.  254 does this by just not going that
// fast...  We want to maybe replan when we get behind, or something.  Maybe
// stop moving the setpoint like our 2018 arm?

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
