#include "frc971/control_loops/drivetrain/line_follow_drivetrain.h"

#include <chrono>

#include "aos/testing/test_shm.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"
#include "frc971/control_loops/drivetrain/trajectory.h"
#include "gflags/gflags.h"
#include "gtest/gtest.h"
#include "third_party/matplotlib-cpp/matplotlibcpp.h"

DEFINE_bool(plot, false, "If true, plot");

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
        drivetrain_(config_),
        velocity_drivetrain_(::std::unique_ptr<StateFeedbackLoop<
            2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
            HybridKalman<2, 2, 2>>>(
            new StateFeedbackLoop<2, 2, 2, double,
                                  StateFeedbackHybridPlant<2, 2, 2>,
                                  HybridKalman<2, 2, 2>>(
                config_.make_hybrid_drivetrain_velocity_loop()))),
        Tlr_to_la_(config_.Tlr_to_la()),
        Tla_to_lr_(config_.Tla_to_lr()) {}

  void set_goal_pose(const Pose &pose) {
    goal_pose_ = pose;
  }

  void RunForTime(chrono::nanoseconds t) {
    const int niter = t / config_.dt;
    for (int ii = 0; ii < niter; ++ii) {
      Iterate();
    }
  }

  void Iterate() {
    ::frc971::control_loops::DrivetrainQueue::Goal goal;
    goal.throttle = driver_model_(state_);
    drivetrain_.SetGoal(goal, goal_pose_);
    drivetrain_.Update(state_);

    ::frc971::control_loops::DrivetrainQueue::Output output;
    drivetrain_.SetOutput(&output);

    EXPECT_LE(::std::abs(output.left_voltage), 12.0 + 1e-6);
    EXPECT_LE(::std::abs(output.right_voltage), 12.0 + 1e-6);

    ::Eigen::Matrix<double, 2, 1> U(output.left_voltage, output.right_voltage);

    state_ = RungeKuttaU(
        [this](const ::Eigen::Matrix<double, 5, 1> &X,
               const ::Eigen::Matrix<double, 2, 1> &U) {
          return ContinuousDynamics(velocity_drivetrain_->plant(), Tlr_to_la_,
                                    X, U);
        },
        state_, U,
        chrono::duration_cast<chrono::duration<double>>(config_.dt).count());
    t_ += config_.dt;

    time_.push_back(chrono::duration_cast<chrono::duration<double>>(
                        t_.time_since_epoch()).count());
    simulation_ul_.push_back(U(0, 0));
    simulation_ur_.push_back(U(1, 0));
    simulation_x_.push_back(state_.x());
    simulation_y_.push_back(state_.y());
  }

  // Check that left/right velocities are near zero and the absolute position is
  // near that of the goal Pose.
  void VerifyNearGoal() const {
    EXPECT_NEAR(goal_pose_.abs_pos().x(), state_(0, 0), 1e-3);
    EXPECT_NEAR(goal_pose_.abs_pos().y(), state_(1, 0), 1e-3);
    // state should be at 0 or PI (we should've come in striaght on or straight
    // backwards).
    const double angle_err =
        ::aos::math::DiffAngle(goal_pose_.abs_theta(), state_(2, 0));
    const double angle_pi_err = ::aos::math::DiffAngle(angle_err, M_PI);
    EXPECT_LT(::std::min(::std::abs(angle_err), ::std::abs(angle_pi_err)),
              1e-3);
    // Left/right velocities are zero:
    EXPECT_NEAR(0.0, state_(3, 0), 1e-3);
    EXPECT_NEAR(0.0, state_(4, 0), 1e-3);
  }

  void CheckGoalTheta(double x, double y, double v, double expected_theta,
                          double expected_thetadot) {
    ::Eigen::Matrix<double, 5, 1> state;
    state << x, y, 0.0, v, v;
    const double theta = drivetrain_.GoalTheta(state);
    const double thetadot = drivetrain_.GoalThetaDot(state);
    EXPECT_EQ(expected_theta, theta) << "x " << x << " y " << y << " v " << v;
    EXPECT_EQ(expected_thetadot, thetadot)
        << "x " << x << " y " << y << " v " << v;
  }

  void CheckGoalThetaDotAtState(double x, double y, double v) {
    ::Eigen::Matrix<double, 5, 1> state;
    state << x, y, 0.0, v, v;
    const double theta = drivetrain_.GoalTheta(state);
    const double thetadot = drivetrain_.GoalThetaDot(state);
    const double dx = v * ::std::cos(theta);
    const double dy = v * ::std::sin(theta);
    constexpr double kEps = 1e-5;
    state(0, 0) += dx * kEps;
    state(1, 0) += dy * kEps;
    const double next_theta = drivetrain_.GoalTheta(state);
    EXPECT_NEAR(thetadot, ::aos::math::DiffAngle(next_theta, theta) / kEps,
                1e-4)
        << "theta: " << theta << " nexttheta: " << next_theta << " x " << x
        << " y " << y << " v " << v;
  }

  void TearDown() override {
    if (FLAGS_plot) {
      matplotlibcpp::figure();
      matplotlibcpp::plot(time_, simulation_ul_, {{"label", "ul"}});
      matplotlibcpp::plot(time_, simulation_ur_, {{"label", "ur"}});
      matplotlibcpp::legend();

      Pose base_pose(&goal_pose_, {-1.0, 0.0, 0.0}, 0.0);
      ::std::vector<double> line_x(
          {base_pose.abs_pos().x(), goal_pose_.abs_pos().x()});
      ::std::vector<double> line_y(
          {base_pose.abs_pos().y(), goal_pose_.abs_pos().y()});
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
  ::Eigen::Matrix<double, 5, 1> state_;

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

  // Current goal pose we are trying to drive to.
  Pose goal_pose_;

  aos::monotonic_clock::time_point t_;

  // Debugging vectors for plotting:
  ::std::vector<double> time_;
  ::std::vector<double> simulation_ul_;
  ::std::vector<double> simulation_ur_;
  ::std::vector<double> simulation_x_;
  ::std::vector<double> simulation_y_;
};

TEST_F(LineFollowDrivetrainTest, ValidGoalThetaDot) {
  for (double x : {1.0, 0.0, -1.0, -2.0, -3.0}) {
    for (double y : {-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0}) {
      for (double v : {-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0}) {
        if (x == 0.0 && y == 0.0) {
          // When x/y are zero, we can encounter singularities. The code should
          // just provide zeros in this case.
          CheckGoalTheta(x, y, v, 0.0, 0.0);
        } else {
          CheckGoalThetaDotAtState(x, y, v);
        }
      }
    }
  }
}

// If the driver commands the robot to keep going, we should just run straight
// off the end and keep going along the line:
TEST_F(LineFollowDrivetrainTest, RunOffEnd) {
  state_ << -1.0, 0.1, 0.0, 0.0, 0.0;
  driver_model_ = [](const ::Eigen::Matrix<double, 5, 1> &) { return 1.0; };
  RunForTime(chrono::seconds(10));
  EXPECT_LT(6.0, state_.x());
  EXPECT_NEAR(0.0, state_.y(), 1e-4);
  EXPECT_NEAR(0.0, state_(2, 0), 1e-4);
  EXPECT_NEAR(1.0, state_(3, 0), 1e-4);
  EXPECT_NEAR(1.0, state_(4, 0), 1e-4);
}

class LineFollowDrivetrainTargetParamTest
    : public LineFollowDrivetrainTest,
      public ::testing::WithParamInterface<Pose> {};
TEST_P(LineFollowDrivetrainTargetParamTest, NonZeroTargetTest) {
  // Start the state at zero and then put the target in a
  state_.setZero();
  driver_model_ = [this](const ::Eigen::Matrix<double, 5, 1> &state) {
    return (state.topRows<2>() - GetParam().abs_pos().topRows<2>()).norm();
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
        ::testing::Values(
            [](const ::Eigen::Matrix<double, 5, 1> &state) {
              return -5.0 * state.x();
            },
            [](const ::Eigen::Matrix<double, 5, 1> &state) {
              return 5.0 * state.x();
            },
            [](const ::Eigen::Matrix<double, 5, 1> &state) {
              return -1.0 * ::std::abs(state.x()) - 0.5 * state.x() * state.x();
            })));

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
