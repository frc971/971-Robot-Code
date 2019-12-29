#include "frc971/control_loops/drivetrain/line_follow_drivetrain.h"

#include <chrono>

#include "aos/testing/test_shm.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"
#include "frc971/control_loops/drivetrain/trajectory.h"
#include "gflags/gflags.h"
#include "gtest/gtest.h"
#include "third_party/matplotlib-cpp/matplotlibcpp.h"

DECLARE_bool(plot);

namespace chrono = ::std::chrono;

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

class LineFollowDrivetrainTest : public ::testing::Test {
 public:
  typedef TypedPose<double> Pose;

  ::aos::testing::TestSharedMemory shm_;

  LineFollowDrivetrainTest()
      : config_(GetTestDrivetrainConfig()),
        drivetrain_(config_, &target_selector_),
        velocity_drivetrain_(
            ::std::unique_ptr<StateFeedbackLoop<
                2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                HybridKalman<2, 2, 2>>>(
                new StateFeedbackLoop<2, 2, 2, double,
                                      StateFeedbackHybridPlant<2, 2, 2>,
                                      HybridKalman<2, 2, 2>>(
                    config_.make_hybrid_drivetrain_velocity_loop()))),
        Tlr_to_la_(config_.Tlr_to_la()),
        Tla_to_lr_(config_.Tla_to_lr()) {}

  void set_goal_pose(const Pose &pose) { target_selector_.set_pose(pose); }

  Pose goal_pose() const { return target_selector_.TargetPose(); }

  void RunForTime(chrono::nanoseconds t) {
    const int niter = t / config_.dt;
    for (int ii = 0; ii < niter; ++ii) {
      Iterate();
    }
  }

  void Iterate() {
    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(1);
    Goal::Builder goal_builder(fbb);
    goal_builder.add_throttle(driver_model_(state_));
    goal_builder.add_controller_type(freeze_target_
                                         ? ControllerType::LINE_FOLLOWER
                                         : ControllerType::POLYDRIVE);
    fbb.Finish(goal_builder.Finish());

    aos::FlatbufferDetachedBuffer<Goal> goal(fbb.Release());

    drivetrain_.SetGoal(t_, &goal.message());
    drivetrain_.Update(t_, state_);

    ::frc971::control_loops::drivetrain::OutputT output;
    EXPECT_EQ(expect_output_, drivetrain_.SetOutput(&output));
    if (!expect_output_) {
      EXPECT_EQ(0.0, output.left_voltage);
      EXPECT_EQ(0.0, output.right_voltage);
    }

    EXPECT_LE(::std::abs(output.left_voltage), 12.0 + 1e-6);
    EXPECT_LE(::std::abs(output.right_voltage), 12.0 + 1e-6);

    ::Eigen::Matrix<double, 2, 1> U(output.left_voltage, output.right_voltage);

    state_ = RungeKuttaU(
        [this](const ::Eigen::Matrix<double, 5, 1> &X,
               const ::Eigen::Matrix<double, 2, 1> &U) {
          return ContinuousDynamics(velocity_drivetrain_->plant(), Tlr_to_la_,
                                    X, U);
        },
        state_, U, ::aos::time::DurationInSeconds(config_.dt));
    t_ += config_.dt;

    time_.push_back(::aos::time::DurationInSeconds(t_.time_since_epoch()));
    simulation_ul_.push_back(U(0, 0));
    simulation_ur_.push_back(U(1, 0));
    simulation_x_.push_back(state_.x());
    simulation_y_.push_back(state_.y());
    simulation_theta_.push_back(state_(3, 0));
  }

  // Check that left/right velocities are near zero and the absolute position is
  // near that of the goal Pose.
  void VerifyNearGoal() const {
    EXPECT_NEAR(goal_pose().abs_pos().x(), state_(0, 0), 1e-2);
    EXPECT_NEAR(goal_pose().abs_pos().y(), state_(1, 0), 1e-2);
    // Left/right velocities are zero:
    EXPECT_NEAR(0.0, state_(3, 0), 1e-2);
    EXPECT_NEAR(0.0, state_(4, 0), 1e-2);
  }

  double GoalTheta(double x, double y, double v, double throttle) {
    flatbuffers::FlatBufferBuilder fbb;
    fbb.ForceDefaults(1);
    Goal::Builder goal_builder(fbb);
    goal_builder.add_throttle(throttle);
    fbb.Finish(goal_builder.Finish());

    aos::FlatbufferDetachedBuffer<Goal> goal(fbb.Release());

    drivetrain_.SetGoal(t_, &goal.message());
    ::Eigen::Matrix<double, 5, 1> state;
    state << x, y, 0.0, v, v;
    return drivetrain_.GoalTheta(state, y, throttle > 0.0 ? 1.0 : -1.0);
  }

  void TearDown() override {
    if (FLAGS_plot) {
      matplotlibcpp::figure();
      matplotlibcpp::plot(time_, simulation_ul_, {{"label", "ul"}});
      matplotlibcpp::plot(time_, simulation_ur_, {{"label", "ur"}});
      matplotlibcpp::plot(time_, simulation_theta_, {{"label", "theta"}});
      matplotlibcpp::legend();

      TypedPose<double> target = goal_pose();
      Pose base_pose(&target, {-1.0, 0.0, 0.0}, 0.0);
      ::std::vector<double> line_x(
          {base_pose.abs_pos().x(), target.abs_pos().x()});
      ::std::vector<double> line_y(
          {base_pose.abs_pos().y(), target.abs_pos().y()});
      matplotlibcpp::figure();
      matplotlibcpp::plot(line_x, line_y, {{"label", "line"}, {"marker", "o"}});
      matplotlibcpp::plot(simulation_x_, simulation_y_,
                          {{"label", "robot"}, {"marker", "o"}});
      matplotlibcpp::legend();

      matplotlibcpp::show();
    }
  }

 protected:
  // Strategy used by the driver to command throttle.
  // This function should return a throttle to apply given the current state.
  // Default to simple proportional gain:
  ::std::function<double(::Eigen::Matrix<double, 5, 1>)> driver_model_ =
      [](const ::Eigen::Matrix<double, 5, 1> &state) {
        return -5.0 * state.x();
      };

  // Current state; [x, y, theta, left_velocity, right_velocity]
  ::Eigen::Matrix<double, 5, 1> state_ = ::Eigen::Matrix<double, 5, 1>::Zero();
  TrivialTargetSelector target_selector_;

  bool freeze_target_ = false;
  bool expect_output_ = true;

 private:
  const DrivetrainConfig<double> config_;

  LineFollowDrivetrain drivetrain_;

  ::std::unique_ptr<
      StateFeedbackLoop<2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                        HybridKalman<2, 2, 2>>>
      velocity_drivetrain_;

  // Transformation matrix from left, right to linear, angular
  const ::Eigen::Matrix<double, 2, 2> Tlr_to_la_;
  // Transformation matrix from linear, angular to left, right
  const ::Eigen::Matrix<double, 2, 2> Tla_to_lr_;

  aos::monotonic_clock::time_point t_;

  // Debugging vectors for plotting:
  ::std::vector<double> time_;
  ::std::vector<double> simulation_ul_;
  ::std::vector<double> simulation_ur_;
  ::std::vector<double> simulation_x_;
  ::std::vector<double> simulation_y_;
  ::std::vector<double> simulation_theta_;
};

TEST_F(LineFollowDrivetrainTest, BasicGoalThetaCheck) {
  for (double x : {1.0, 0.0, -1.0, -2.0, -3.0}) {
    for (double y : {-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0}) {
      for (double v : {-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0}) {
        for (double throttle : {-1.0, 1.0}) {
          target_selector_.set_target_radius(0.0);
          const double zero_rad_theta = GoalTheta(x, y, v, throttle);
          EXPECT_NEAR(
              0.0,
              ::aos::math::DiffAngle((throttle > 0.0 ? M_PI : 0.0) +
                                         ::std::atan2(y, ::std::min(-0.01, x)),
                                     zero_rad_theta),
              1e-14);
          target_selector_.set_target_radius(0.05);
          const double small_rad_theta = GoalTheta(x, y, v, throttle);
          if (y > 0) {
            EXPECT_LT(small_rad_theta, zero_rad_theta);
          } else if (y == 0) {
            EXPECT_NEAR(0.0,
                        ::aos::math::DiffAngle(small_rad_theta, zero_rad_theta),
                        1e-14);
          } else {
            EXPECT_GT(small_rad_theta, zero_rad_theta);
          }
        }
      }
    }
  }
}

// If the driver commands the robot to keep going, we should just run straight
// off the end and keep going along the line:
TEST_F(LineFollowDrivetrainTest, RunOffEnd) {
  freeze_target_ = true;
  state_ << -1.0, 0.0, 0.0, 0.0, 0.0;
  // TODO(james): This test currently relies on the scalar on the throttle to
  // velocity conversion being 4.0. This should probably be moved out into a
  // config.
  driver_model_ = [](const ::Eigen::Matrix<double, 5, 1> &) { return 0.25; };
  RunForTime(chrono::seconds(10));
  EXPECT_LT(6.0, state_.x());
  EXPECT_NEAR(0.0, state_.y(), 1e-4);
  EXPECT_NEAR(0.0, state_(2, 0), 1e-4);
  EXPECT_NEAR(1.0, state_(3, 0), 1e-4);
  EXPECT_NEAR(1.0, state_(4, 0), 1e-4);
}

// Tests that the outputs are not set when there is no valid target.
TEST_F(LineFollowDrivetrainTest, DoesntHaveTarget) {
  state_.setZero();
  target_selector_.set_has_target(false);
  expect_output_ = false;
  // There are checks in Iterate for the logic surrounding SetOutput().
  RunForTime(::std::chrono::seconds(5));
  // The robot should not have gone anywhere.
  EXPECT_NEAR(0.0, state_.squaredNorm(), 1e-25);
}

// Tests that, when we set the controller type to be line following, the target
// selection freezes.
TEST_F(LineFollowDrivetrainTest, FreezeOnControllerType) {
  state_.setZero();
  set_goal_pose({{0.0, 0.0, 0.0}, 0.0});
  // Do one iteration to get the target into the drivetrain:
  Iterate();

  freeze_target_ = true;

  // Set a goal pose that we should not go to.
  set_goal_pose({{1.0, 1.0, 0.0}, M_PI_2});

  RunForTime(::std::chrono::seconds(5));
  EXPECT_NEAR(0.0, state_.squaredNorm(), 1e-25)
      << "Expected state of zero, got: " << state_.transpose();
}

// Tests that when we freeze the controller without having acquired a target, we
// don't do anything until a target arrives.
TEST_F(LineFollowDrivetrainTest, FreezeWithoutAcquiringTarget) {
  freeze_target_ = true;
  target_selector_.set_has_target(false);
  expect_output_ = false;
  state_.setZero();

  RunForTime(::std::chrono::seconds(5));

  // Nothing should've happened
  EXPECT_NEAR(0.0, state_.squaredNorm(), 1e-25);

  // Now, provide a target:
  target_selector_.set_has_target(true);
  set_goal_pose({{1.0, 2.0, 0.0}, M_PI_2});
  driver_model_ = [this](const ::Eigen::Matrix<double, 5, 1> &state) {
    return -(state.y() - goal_pose().abs_pos().y());
  };
  expect_output_ = true;

  Iterate();

  // And remove the target, to ensure that we keep going if we do loose the
  // target:
  target_selector_.set_has_target(false);

  RunForTime(::std::chrono::seconds(15));

  VerifyNearGoal();
}

class LineFollowDrivetrainTargetParamTest
    : public LineFollowDrivetrainTest,
      public ::testing::WithParamInterface<Pose> {};
TEST_P(LineFollowDrivetrainTargetParamTest, NonZeroTargetTest) {
  freeze_target_ = true;
  target_selector_.set_has_target(true);
  // Start the state at zero and then put the target in a
  state_.setZero();
  driver_model_ = [this](const ::Eigen::Matrix<double, 5, 1> &state) {
    return 0.2 *
           (state.topRows<2>() - GetParam().abs_pos().topRows<2>()).norm();
  };
  set_goal_pose(GetParam());
  RunForTime(chrono::seconds(10));
  VerifyNearGoal();
}
INSTANTIATE_TEST_CASE_P(TargetPosTest, LineFollowDrivetrainTargetParamTest,
                        ::testing::Values(Pose({0.0, 0.0, 0.0}, 0.0),
                                          Pose({1.0, 0.0, 0.0}, 0.0),
                                          Pose({3.0, 1.0, 0.0}, 0.0),
                                          Pose({3.0, 0.0, 0.0}, 0.5),
                                          Pose({3.0, 0.0, 0.0}, -0.5),
                                          Pose({-3.0, -1.0, 0.0}, -2.5)));

class LineFollowDrivetrainParamTest
    : public LineFollowDrivetrainTest,
      public ::testing::WithParamInterface<::std::tuple<
          ::Eigen::Matrix<double, 5, 1>,
          ::std::function<double(const ::Eigen::Matrix<double, 5, 1> &)>>> {};

TEST_P(LineFollowDrivetrainParamTest, VaryPositionAndModel) {
  state_ = ::std::get<0>(GetParam());
  driver_model_ = ::std::get<1>(GetParam());
  RunForTime(chrono::seconds(10));
  VerifyNearGoal();
}

INSTANTIATE_TEST_CASE_P(
    PositionAndModelTest, LineFollowDrivetrainParamTest,
    ::testing::Combine(
        ::testing::Values(
            (::Eigen::Matrix<double, 5, 1>() << -1.0, 0.0, 0.0, 0.0, 0.0)
                .finished(),
            (::Eigen::Matrix<double, 5, 1>() << -2.0, 0.0, 0.0, 0.0, 0.0)
                .finished(),
            (::Eigen::Matrix<double, 5, 1>() << -2.0, 0.4, 0.0, 0.0, 0.0)
                .finished(),
            (::Eigen::Matrix<double, 5, 1>() << -2.0, -0.4, 0.0, 0.0, 0.0)
                .finished(),
            (::Eigen::Matrix<double, 5, 1>() << -2.0, 0.0, 0.5, 0.0, 0.0)
                .finished(),
            (::Eigen::Matrix<double, 5, 1>() << -2.0, 0.0, -0.5, 0.0, 0.0)
                .finished(),
            (::Eigen::Matrix<double, 5, 1>() << -2.0, 1.0, 0.5, 0.0, 0.0)
                .finished(),
            (::Eigen::Matrix<double, 5, 1>() << -2.0, 1.0, -0.5, 0.0, 0.0)
                .finished(),
            (::Eigen::Matrix<double, 5, 1>() << -2.0, -1.0, 0.5, 0.0, 0.0)
                .finished(),
            (::Eigen::Matrix<double, 5, 1>() << -2.0, -1.0, -0.5, 0.0, 0.0)
                .finished(),
            (::Eigen::Matrix<double, 5, 1>() << -2.0, 1.0, -2.0, 0.0, 0.0)
                .finished(),
            (::Eigen::Matrix<double, 5, 1>() << -2.0, -1.0, 2.0, 0.0, 0.0)
                .finished(),
            (::Eigen::Matrix<double, 5, 1>() << -2.0, -1.0, 12.0, 0.0, 0.0)
                .finished(),
            (::Eigen::Matrix<double, 5, 1>() << -2.0, -1.0, 0.0, 0.5, -0.5)
                .finished()),
        ::testing::Values([](const ::Eigen::Matrix<double, 5, 1>
                                 &state) { return -1.0 * state.x(); },
                          [](const ::Eigen::Matrix<double, 5, 1> &state) {
                            return 1.0 * state.x();
                          },
                          [](const ::Eigen::Matrix<double, 5, 1> &state) {
                            return -0.25 * ::std::abs(state.x()) -
                                   0.125 * state.x() * state.x();
                          })));

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
