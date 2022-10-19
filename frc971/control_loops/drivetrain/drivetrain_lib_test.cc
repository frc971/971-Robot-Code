#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/events/event_loop.h"
#include "aos/events/logging/log_writer.h"
#include "aos/time/time.h"
#include "frc971/control_loops/coerce_goal.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "frc971/control_loops/drivetrain/trajectory_generator.h"
#include "frc971/control_loops/polytope.h"
#include "frc971/queues/gyro_generated.h"
#include "gflags/gflags.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

DEFINE_string(output_file, "",
              "If set, logs all channels to the provided logfile.");

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

class DrivetrainTest : public ::frc971::testing::ControlLoopTest {
 protected:
  DrivetrainTest()
      : ::frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig(
                "frc971/control_loops/drivetrain/simulation_config.json"),
            GetTestDrivetrainConfig().dt),
        test_event_loop_(MakeEventLoop("test")),
        drivetrain_goal_sender_(
            test_event_loop_->MakeSender<Goal>("/drivetrain")),
        trajectory_goal_sender_(
            test_event_loop_->MakeSender<SplineGoal>("/drivetrain")),
        drivetrain_goal_fetcher_(
            test_event_loop_->MakeFetcher<Goal>("/drivetrain")),
        drivetrain_status_fetcher_(
            test_event_loop_->MakeFetcher<Status>("/drivetrain")),
        drivetrain_output_fetcher_(
            test_event_loop_->MakeFetcher<Output>("/drivetrain")),
        localizer_control_sender_(
            test_event_loop_->MakeSender<LocalizerControl>("/drivetrain")),
        drivetrain_event_loop_(MakeEventLoop("drivetrain")),
        trajectory_generator_event_loop_(MakeEventLoop("trajectory_generator")),
        dt_config_(GetTestDrivetrainConfig()),
        localizer_(drivetrain_event_loop_.get(), dt_config_),
        drivetrain_(dt_config_, drivetrain_event_loop_.get(), &localizer_),
        trajectory_generator_(trajectory_generator_event_loop_.get(),
                              dt_config_),
        drivetrain_plant_event_loop_(MakeEventLoop("drivetrain_plant")),
        drivetrain_plant_(drivetrain_plant_event_loop_.get(), dt_config_) {
    // Too many tests care...
    set_send_delay(chrono::nanoseconds(0));
    set_battery_voltage(12.0);

    if (!FLAGS_output_file.empty()) {
      unlink(FLAGS_output_file.c_str());
      logger_event_loop_ = MakeEventLoop("logger");
      logger_ = std::make_unique<aos::logger::Logger>(logger_event_loop_.get());
      logger_->StartLoggingOnRun(FLAGS_output_file);
    }

    // Run for enough time to allow the gyro/imu zeroing code to run.
    RunFor(std::chrono::seconds(15));
    CHECK(drivetrain_status_fetcher_.Fetch());
    EXPECT_TRUE(CHECK_NOTNULL(drivetrain_status_fetcher_->zeroing())->zeroed());
  }
  virtual ~DrivetrainTest() {}

  void TearDown() override { drivetrain_plant_.MaybePlot(); }

  void VerifyNearGoal() {
    drivetrain_goal_fetcher_.Fetch();
    EXPECT_NEAR(drivetrain_goal_fetcher_->left_goal(),
                drivetrain_plant_.GetLeftPosition(), 1e-2);
    EXPECT_NEAR(drivetrain_goal_fetcher_->right_goal(),
                drivetrain_plant_.GetRightPosition(), 1e-2);
  }

  void VerifyNearPosition(double x, double y, double eps = 1e-2) {
    auto actual = drivetrain_plant_.GetPosition();
    EXPECT_NEAR(actual(0), x, eps);
    EXPECT_NEAR(actual(1), y, eps);
  }

  void VerifyNearSplineGoal() {
    drivetrain_status_fetcher_.Fetch();
    const double expected_x =
        CHECK_NOTNULL(drivetrain_status_fetcher_->trajectory_logging())->x();
    const double expected_y =
        CHECK_NOTNULL(drivetrain_status_fetcher_->trajectory_logging())->y();
    const double estimated_x = drivetrain_status_fetcher_->x();
    const double estimated_y = drivetrain_status_fetcher_->y();
    const ::Eigen::Vector2d actual = drivetrain_plant_.GetPosition();
    EXPECT_NEAR(estimated_x, expected_x, spline_control_tolerance_);
    EXPECT_NEAR(estimated_y, expected_y, spline_control_tolerance_);
    EXPECT_NEAR(actual(0), estimated_x, spline_estimate_tolerance_);
    EXPECT_NEAR(actual(1), estimated_y, spline_estimate_tolerance_);
  }

  void WaitForTrajectoryExecution() {
    do {
      RunFor(dt());
      EXPECT_TRUE(drivetrain_status_fetcher_.Fetch());
    } while (!CHECK_NOTNULL(drivetrain_status_fetcher_->trajectory_logging())
                  ->is_executed());
  }

  void VerifyDownEstimator() {
    EXPECT_TRUE(drivetrain_status_fetcher_.Fetch());
    // TODO(james): Handle Euler angle singularities...
    const double down_estimator_yaw =
        CHECK_NOTNULL(drivetrain_status_fetcher_->down_estimator())->yaw();
    const double localizer_yaw = drivetrain_status_fetcher_->theta();
    EXPECT_LT(std::abs(aos::math::DiffAngle(down_estimator_yaw, localizer_yaw)),
              1e-2);
    const double true_yaw = (drivetrain_plant_.GetRightPosition() -
                             drivetrain_plant_.GetLeftPosition()) /
                            (dt_config_.robot_radius * 2.0);
    EXPECT_LT(std::abs(aos::math::DiffAngle(down_estimator_yaw, true_yaw)),
              1e-4);
    // We don't currently simulate any pitch or roll, so we shouldn't be
    // reporting any.
    EXPECT_NEAR(
        0, drivetrain_status_fetcher_->down_estimator()->longitudinal_pitch(),
        1e-10);
    EXPECT_NEAR(0,
                drivetrain_status_fetcher_->down_estimator()->lateral_pitch(),
                1e-10);
  }

  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::Sender<::frc971::control_loops::drivetrain::Goal>
      drivetrain_goal_sender_;
  ::aos::Sender<SplineGoal> trajectory_goal_sender_;
  ::aos::Fetcher<::frc971::control_loops::drivetrain::Goal>
      drivetrain_goal_fetcher_;
  ::aos::Fetcher<::frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;
  ::aos::Fetcher<::frc971::control_loops::drivetrain::Output>
      drivetrain_output_fetcher_;
  ::aos::Sender<LocalizerControl> localizer_control_sender_;

  ::std::unique_ptr<::aos::EventLoop> drivetrain_event_loop_;
  ::std::unique_ptr<::aos::EventLoop> trajectory_generator_event_loop_;
  const DrivetrainConfig<double> dt_config_;
  DeadReckonEkf localizer_;
  // Create a loop and simulation plant.
  DrivetrainLoop drivetrain_;
  TrajectoryGenerator trajectory_generator_;

  ::std::unique_ptr<::aos::EventLoop> drivetrain_plant_event_loop_;
  DrivetrainSimulation drivetrain_plant_;

  std::unique_ptr<aos::EventLoop> logger_event_loop_;
  std::unique_ptr<aos::logger::DetachedBufferWriter> log_buffer_writer_;
  std::unique_ptr<aos::logger::Logger> logger_;

  double spline_estimate_tolerance_ = 0.05;
  double spline_control_tolerance_ = 0.05;
};

// Tests that the drivetrain converges on a goal.
TEST_F(DrivetrainTest, ConvergesCorrectly) {
  SetEnabled(true);
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::MOTION_PROFILE);
    goal_builder.add_left_goal(-1.0);
    goal_builder.add_right_goal(1.0);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  RunFor(chrono::seconds(2));
  VerifyNearGoal();
  VerifyDownEstimator();
}

// Tests that the drivetrain disables itself when the IMU errors.
TEST_F(DrivetrainTest, DisablesOnImuError) {
  SetEnabled(true);
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::MOTION_PROFILE);
    goal_builder.add_left_goal(-1.0);
    goal_builder.add_right_goal(1.0);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  // Sanity check that the drivetrain is indeed commanding voltage while the IMU
  // is healthy.
  for (int i = 0; i < 50; ++i) {
    RunFor(dt());
    ASSERT_TRUE(drivetrain_output_fetcher_.Fetch());
    EXPECT_NE(0.0, drivetrain_output_fetcher_->left_voltage());
    EXPECT_NE(0.0, drivetrain_output_fetcher_->right_voltage());
  }

  // Fault the IMU and confirm that we disable the outputs.
  drivetrain_plant_.set_imu_faulted(true);

  // Ensure the fault has time to propagate.
  RunFor(2 * dt());

  for (int i = 0; i < 500; ++i) {
    RunFor(dt());
    ASSERT_TRUE(drivetrain_output_fetcher_.Fetch());
    EXPECT_EQ(0.0, drivetrain_output_fetcher_->left_voltage());
    EXPECT_EQ(0.0, drivetrain_output_fetcher_->right_voltage());
  }
}

// Tests that the drivetrain converges on a goal when under the effect of a
// voltage offset/disturbance.
TEST_F(DrivetrainTest, ConvergesWithVoltageError) {
  SetEnabled(true);
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::MOTION_PROFILE);
    goal_builder.add_left_goal(-1.0);
    goal_builder.add_right_goal(1.0);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  drivetrain_plant_.set_left_voltage_offset(1.0);
  drivetrain_plant_.set_right_voltage_offset(1.0);
  RunFor(chrono::milliseconds(1500));
  VerifyNearGoal();
}

// Tests that it survives disabling.
TEST_F(DrivetrainTest, SurvivesDisabling) {
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::MOTION_PROFILE);
    goal_builder.add_left_goal(-1.0);
    goal_builder.add_right_goal(1.0);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  for (int i = 0; i < 500; ++i) {
    if (i > 20 && i < 200) {
      SetEnabled(false);
    } else {
      SetEnabled(true);
    }
    RunFor(dt());
  }
  VerifyNearGoal();
}

// Tests that never having a goal, but having driver's station messages, doesn't
// break.
TEST_F(DrivetrainTest, NoGoalWithRobotState) {
  SetEnabled(true);
  RunFor(chrono::milliseconds(100));
}

// Tests that the robot successfully drives straight forward.
// This used to not work due to a U-capping bug.
TEST_F(DrivetrainTest, DriveStraightForward) {
  SetEnabled(true);
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::MOTION_PROFILE);
    goal_builder.add_left_goal(4.0);
    goal_builder.add_right_goal(4.0);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  for (int i = 0; i < 500; ++i) {
    RunFor(dt());
    ASSERT_TRUE(drivetrain_output_fetcher_.Fetch());
    EXPECT_NEAR(drivetrain_output_fetcher_->left_voltage(),
                drivetrain_output_fetcher_->right_voltage(), 1e-4);
    EXPECT_GT(drivetrain_output_fetcher_->left_voltage(), -11);
    EXPECT_GT(drivetrain_output_fetcher_->right_voltage(), -11);
  }
  VerifyNearGoal();
}

// Tests that the robot successfully drives close to straight.
// This used to fail in simulation due to libcdd issues with U-capping.
TEST_F(DrivetrainTest, DriveAlmostStraightForward) {
  SetEnabled(true);
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::MOTION_PROFILE);
    goal_builder.add_left_goal(4.0);
    goal_builder.add_right_goal(3.9);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  for (int i = 0; i < 500; ++i) {
    RunFor(dt());
    ASSERT_TRUE(drivetrain_output_fetcher_.Fetch());
    EXPECT_GT(drivetrain_output_fetcher_->left_voltage(), -11);
    EXPECT_GT(drivetrain_output_fetcher_->right_voltage(), -11);
  }
  VerifyNearGoal();
}

// Tests that converting from a left, right position to a distance, angle
// coordinate system and back returns the same answer.
TEST_F(DrivetrainTest, LinearToAngularAndBack) {
  const double width = dt_config_.robot_radius * 2.0;

  Eigen::Matrix<double, 7, 1> state;
  state << 2, 3, 4, 5, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> linear = dt_config_.LeftRightToLinear(state);

  EXPECT_NEAR(3.0, linear(0, 0), 1e-6);
  EXPECT_NEAR(4.0, linear(1, 0), 1e-6);

  Eigen::Matrix<double, 2, 1> angular = dt_config_.LeftRightToAngular(state);

  EXPECT_NEAR(2.0 / width, angular(0, 0), 1e-6);
  EXPECT_NEAR(2.0 / width, angular(1, 0), 1e-6);

  Eigen::Matrix<double, 4, 1> back_state =
      dt_config_.AngularLinearToLeftRight(linear, angular);

  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(state(i, 0), back_state(i, 0), 1e-8);
  }
}

// Tests that a linear motion profile succeeds.
TEST_F(DrivetrainTest, ProfileStraightForward) {
  SetEnabled(true);
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    ProfileParameters::Builder linear_builder =
        builder.MakeBuilder<ProfileParameters>();
    linear_builder.add_max_velocity(1.0);
    linear_builder.add_max_acceleration(3.0);
    flatbuffers::Offset<ProfileParameters> linear_offset =
        linear_builder.Finish();

    ProfileParameters::Builder angular_builder =
        builder.MakeBuilder<ProfileParameters>();
    angular_builder.add_max_velocity(1.0);
    angular_builder.add_max_acceleration(3.0);
    flatbuffers::Offset<ProfileParameters> angular_offset =
        angular_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::MOTION_PROFILE);
    goal_builder.add_left_goal(4.0);
    goal_builder.add_right_goal(4.0);
    goal_builder.add_linear(linear_offset);
    goal_builder.add_angular(angular_offset);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  const auto start_time = monotonic_now();
  while (monotonic_now() < start_time + chrono::seconds(6)) {
    RunFor(dt());
    ASSERT_TRUE(drivetrain_output_fetcher_.Fetch());
    EXPECT_NEAR(drivetrain_output_fetcher_->left_voltage(),
                drivetrain_output_fetcher_->right_voltage(), 1e-4);
    EXPECT_GT(drivetrain_output_fetcher_->left_voltage(), -6);
    EXPECT_GT(drivetrain_output_fetcher_->right_voltage(), -6);
    EXPECT_LT(drivetrain_output_fetcher_->left_voltage(), 6);
    EXPECT_LT(drivetrain_output_fetcher_->right_voltage(), 6);
  }
  VerifyNearGoal();
}

// Tests that an angular motion profile succeeds.
TEST_F(DrivetrainTest, ProfileTurn) {
  SetEnabled(true);
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    ProfileParameters::Builder linear_builder =
        builder.MakeBuilder<ProfileParameters>();
    linear_builder.add_max_velocity(1.0);
    linear_builder.add_max_acceleration(3.0);
    flatbuffers::Offset<ProfileParameters> linear_offset =
        linear_builder.Finish();

    ProfileParameters::Builder angular_builder =
        builder.MakeBuilder<ProfileParameters>();
    angular_builder.add_max_velocity(1.0);
    angular_builder.add_max_acceleration(3.0);
    flatbuffers::Offset<ProfileParameters> angular_offset =
        angular_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::MOTION_PROFILE);
    goal_builder.add_left_goal(-1.0);
    goal_builder.add_right_goal(1.0);
    goal_builder.add_linear(linear_offset);
    goal_builder.add_angular(angular_offset);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  const auto start_time = monotonic_now();
  while (monotonic_now() < start_time + chrono::seconds(6)) {
    RunFor(dt());
    ASSERT_TRUE(drivetrain_output_fetcher_.Fetch());
    EXPECT_NEAR(drivetrain_output_fetcher_->left_voltage(),
                -drivetrain_output_fetcher_->right_voltage(), 1e-4);
    EXPECT_GT(drivetrain_output_fetcher_->left_voltage(), -6);
    EXPECT_GT(drivetrain_output_fetcher_->right_voltage(), -6);
    EXPECT_LT(drivetrain_output_fetcher_->left_voltage(), 6);
    EXPECT_LT(drivetrain_output_fetcher_->right_voltage(), 6);
  }
  VerifyNearGoal();
}

// Tests that a mixed turn drive saturated profile succeeds.
TEST_F(DrivetrainTest, SaturatedTurnDrive) {
  SetEnabled(true);
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    ProfileParameters::Builder linear_builder =
        builder.MakeBuilder<ProfileParameters>();
    linear_builder.add_max_velocity(6.0);
    linear_builder.add_max_acceleration(4.0);
    flatbuffers::Offset<ProfileParameters> linear_offset =
        linear_builder.Finish();

    ProfileParameters::Builder angular_builder =
        builder.MakeBuilder<ProfileParameters>();
    angular_builder.add_max_velocity(2.0);
    angular_builder.add_max_acceleration(4.0);
    flatbuffers::Offset<ProfileParameters> angular_offset =
        angular_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::MOTION_PROFILE);
    goal_builder.add_left_goal(5.0);
    goal_builder.add_right_goal(4.0);
    goal_builder.add_linear(linear_offset);
    goal_builder.add_angular(angular_offset);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  const auto start_time = monotonic_now();
  while (monotonic_now() < start_time + chrono::seconds(3)) {
    RunFor(dt());
    ASSERT_TRUE(drivetrain_output_fetcher_.Fetch());
  }
  VerifyNearGoal();
}

// Tests that being in teleop drive for a bit and then transitioning to closed
// drive profiles nicely.
TEST_F(DrivetrainTest, OpenLoopThenClosed) {
  SetEnabled(true);
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::POLYDRIVE);
    goal_builder.add_wheel(0.0);
    goal_builder.add_throttle(1.0);
    goal_builder.add_highgear(true);
    goal_builder.add_quickturn(false);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::POLYDRIVE);
    goal_builder.add_wheel(0.0);
    goal_builder.add_throttle(-0.3);
    goal_builder.add_highgear(true);
    goal_builder.add_quickturn(false);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(1));

  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::POLYDRIVE);
    goal_builder.add_wheel(0.0);
    goal_builder.add_throttle(0.0);
    goal_builder.add_highgear(true);
    goal_builder.add_quickturn(false);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(10));

  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    ProfileParameters::Builder linear_builder =
        builder.MakeBuilder<ProfileParameters>();
    linear_builder.add_max_velocity(1.0);
    linear_builder.add_max_acceleration(2.0);
    flatbuffers::Offset<ProfileParameters> linear_offset =
        linear_builder.Finish();

    ProfileParameters::Builder angular_builder =
        builder.MakeBuilder<ProfileParameters>();
    angular_builder.add_max_velocity(1.0);
    angular_builder.add_max_acceleration(2.0);
    flatbuffers::Offset<ProfileParameters> angular_offset =
        angular_builder.Finish();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::MOTION_PROFILE);
    goal_builder.add_left_goal(5.0);
    goal_builder.add_right_goal(4.0);
    goal_builder.add_linear(linear_offset);
    goal_builder.add_angular(angular_offset);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  const auto end_time = monotonic_now() + chrono::seconds(4);
  while (monotonic_now() < end_time) {
    RunFor(dt());
    ASSERT_TRUE(drivetrain_output_fetcher_.Fetch());
    EXPECT_GT(drivetrain_output_fetcher_->left_voltage(), -6);
    EXPECT_GT(drivetrain_output_fetcher_->right_voltage(), -6);
    EXPECT_LT(drivetrain_output_fetcher_->left_voltage(), 6);
    EXPECT_LT(drivetrain_output_fetcher_->right_voltage(), 6);
  }
  VerifyNearGoal();
}

// Tests that simple spline converges on a goal.
TEST_F(DrivetrainTest, SplineSimple) {
  SetEnabled(true);
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.25, 0.5, 0.5, 0.75, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.0, 0.25, 0.75, 1.0, 1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    EXPECT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  RunFor(dt());

  // Send the start goal
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(1);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(2000));
  VerifyNearSplineGoal();
}

// Tests that we can drive a spline backwards.
TEST_F(DrivetrainTest, SplineSimpleBackwards) {
  SetEnabled(true);
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>(
            {0.0, -0.25, -0.5, -0.5, -0.75, -1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>(
            {0.0, 0.0, -0.25, -0.75, -1.0, -1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(true);
    spline_goal_builder.add_spline(multispline_offset);

    EXPECT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  RunFor(dt());

  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(1);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  // Check that we are right on the spline at the start (otherwise the feedback
  // will tend to correct for going the wrong direction).
  for (int ii = 0; ii < 10; ++ii) {
    RunFor(chrono::milliseconds(100));
    VerifyNearSplineGoal();
  }

  WaitForTrajectoryExecution();

  VerifyNearSplineGoal();
  // Check that we are pointed the right direction:
  drivetrain_status_fetcher_.Fetch();
  auto actual = drivetrain_plant_.state();
  const double expected_theta =
      CHECK_NOTNULL(drivetrain_status_fetcher_->trajectory_logging())->theta();
  // As a sanity check, compare both against absolute angle and the spline's
  // goal angle.
  EXPECT_NEAR(0.0, ::aos::math::DiffAngle(actual(2), 0.0), 2e-2);
  EXPECT_NEAR(0.0, ::aos::math::DiffAngle(actual(2), expected_theta), 2e-2);
}

// Tests that simple spline with a single goal message.
TEST_F(DrivetrainTest, SplineSingleGoal) {
  SetEnabled(true);
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.25, 0.5, 0.5, 0.75, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.0, 0.25, 0.75, 1.0, 1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    EXPECT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(1);
    EXPECT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(2000));
  VerifyNearSplineGoal();
}

// Tests that a trajectory can be stopped in the middle.
TEST_F(DrivetrainTest, SplineStop) {
  SetEnabled(true);
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.25, 0.5, 0.5, 0.75, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.0, 0.25, 0.75, 1.0, 1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(1);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(500));
  drivetrain_status_fetcher_.Fetch();

  // Now stop.
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(0);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  for (int i = 0; i < 100; ++i) {
    RunFor(dt());
    ASSERT_TRUE(drivetrain_output_fetcher_.Fetch());
    EXPECT_EQ(0.0, drivetrain_output_fetcher_->left_voltage());
    EXPECT_EQ(0.0, drivetrain_output_fetcher_->right_voltage());
    // The goal should be null after stopping.
    ASSERT_TRUE(drivetrain_status_fetcher_.Fetch());
    EXPECT_FALSE(CHECK_NOTNULL(drivetrain_status_fetcher_->trajectory_logging())
                     ->has_x());
    EXPECT_FALSE(CHECK_NOTNULL(drivetrain_status_fetcher_->trajectory_logging())
                     ->has_y());
  }
}

// Tests that a spline can't be restarted.
TEST_F(DrivetrainTest, SplineRestart) {
  SetEnabled(true);
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.25, 0.5, 0.5, 0.75, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.0, 0.25, 0.75, 1.0, 1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();

    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(1);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(500));
  drivetrain_status_fetcher_.Fetch();

  // Send a stop goal.
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(0);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  RunFor(chrono::milliseconds(500));

  // Send a restart.
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(1);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  RunFor(chrono::milliseconds(2000));

  // The goal should be empty.
  drivetrain_status_fetcher_.Fetch();
  EXPECT_FALSE(
      CHECK_NOTNULL(drivetrain_status_fetcher_->trajectory_logging())->has_x());
  EXPECT_FALSE(
      CHECK_NOTNULL(drivetrain_status_fetcher_->trajectory_logging())->has_y());
}

class DrivetrainBackwardsParamTest
    : public DrivetrainTest,
      public ::testing::WithParamInterface<bool> {};

// Tests that simple spline converges when it doesn't start where it thinks.
TEST_P(DrivetrainBackwardsParamTest, SplineOffset) {
  SetEnabled(true);
  if (GetParam()) {
    // Turn the robot around if we are backwards.
    (*drivetrain_plant_.mutable_state())(2) += M_PI;

    auto builder = localizer_control_sender_.MakeBuilder();
    LocalizerControl::Builder localizer_control_builder =
        builder.MakeBuilder<LocalizerControl>();
    localizer_control_builder.add_x(drivetrain_plant_.state()(0));
    localizer_control_builder.add_y(drivetrain_plant_.state()(1));
    localizer_control_builder.add_theta(drivetrain_plant_.state()(2));
    ASSERT_EQ(builder.Send(localizer_control_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.2, 0.25, 0.5, 0.5, 0.75, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.2, 0.0, 0.25, 0.75, 1.0, 1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(GetParam());
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  RunFor(dt());

  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(1);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(5000));
  VerifyNearSplineGoal();
}

INSTANTIATE_TEST_SUITE_P(DriveSplinesForwardsAndBackwards,
                         DrivetrainBackwardsParamTest,
                         ::testing::Values(false, true));

// Tests that simple spline converges when it starts to the side of where it
// thinks.
TEST_F(DrivetrainTest, SplineSideOffset) {
  SetEnabled(true);
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.25, 0.5, 0.5, 0.75, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.5, 0.0, 0.25, 0.75, 1.0, 1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  RunFor(dt());

  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(1);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(5000));

  spline_control_tolerance_ = 0.1;
  spline_estimate_tolerance_ = 0.1;
  VerifyNearSplineGoal();
}

// Tests that simple spline converges when we introduce a straight voltage
// error.
// TODO(james): Reenable this once we decide what to do with the voltage error
// terms.
TEST_F(DrivetrainTest, SplineVoltageError) {
  SetEnabled(true);
  drivetrain_plant_.set_left_voltage_offset(1.0);
  drivetrain_plant_.set_right_voltage_offset(0.5);
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.25, 0.5, 0.5, 0.75, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.0, 0.25, 0.75, 1.0, 1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  RunFor(dt());

  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(1);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(5000));
  // Since the voltage error compensation is disabled, expect that we will have
  // *failed* to reach our goal.
  drivetrain_status_fetcher_.Fetch();
  const double expected_x =
      CHECK_NOTNULL(drivetrain_status_fetcher_->trajectory_logging())->x();
  const double expected_y =
      CHECK_NOTNULL(drivetrain_status_fetcher_->trajectory_logging())->y();
  const double estimated_x = drivetrain_status_fetcher_->x();
  const double estimated_y = drivetrain_status_fetcher_->y();
  const ::Eigen::Vector2d actual = drivetrain_plant_.GetPosition();
  // Expect the x position comparison to fail; everything else to succeed.
  spline_estimate_tolerance_ = 0.11;
  spline_control_tolerance_ = 0.11;
  EXPECT_GT(std::abs(estimated_x - expected_x), spline_control_tolerance_);
  EXPECT_NEAR(estimated_y, expected_y, spline_control_tolerance_);
  EXPECT_NEAR(actual(0), estimated_x, spline_estimate_tolerance_);
  EXPECT_NEAR(actual(1), estimated_y, spline_estimate_tolerance_);
}

// Tests that a multispline converges on a goal.
TEST_F(DrivetrainTest, MultiSpline) {
  SetEnabled(true);
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>(
            {0.0, 0.25, 0.5, 0.5, 0.75, 1.0, 1.25, 1.5, 1.5, 1.25, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>(
            {0.0, 0.0, 0.25, 0.75, 1.0, 1.0, 1.0, 1.25, 1.5, 1.75, 2.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(2);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  RunFor(dt());

  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(1);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(4000));
  VerifyNearSplineGoal();
}

// Tests that several splines converges on a goal.
TEST_F(DrivetrainTest, SequentialSplines) {
  SetEnabled(true);
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.25, 0.5, 0.5, 0.75, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.0, 0.25, 0.75, 1.0, 1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  RunFor(dt());

  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(1);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  WaitForTrajectoryExecution();

  VerifyNearSplineGoal();

  // Second spline.
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({1.0, 1.25, 1.5, 1.5, 1.25, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({1.0, 1.0, 1.25, 1.5, 1.75, 2.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(2);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  RunFor(dt());

  // And then start it.
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(2);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(2000));
  VerifyNearSplineGoal();
}

// Tests that a second spline will run if the first is stopped.
TEST_F(DrivetrainTest, SplineStopFirst) {
  SetEnabled(true);
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.25, 0.5, 0.5, 0.75, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.0, 0.25, 0.75, 1.0, 1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(1);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(2000));
  {
    drivetrain_status_fetcher_.Fetch();
    EXPECT_TRUE(
        drivetrain_status_fetcher_->trajectory_logging()->is_executing());
    EXPECT_FALSE(
        drivetrain_status_fetcher_->trajectory_logging()->is_executed());
  }

  // Stop goal
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(0);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  RunFor(chrono::milliseconds(500));

  // Second spline goal.
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({1.0, 1.25, 1.5, 1.5, 1.25, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({1.0, 1.0, 1.25, 1.5, 1.75, 2.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(2);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(2);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  WaitForTrajectoryExecution();
  spline_control_tolerance_ = 0.1;
  spline_estimate_tolerance_ = 0.15;
  VerifyNearSplineGoal();
}

// Tests that we can run a second spline after having planned but never executed
// the first spline.
TEST_F(DrivetrainTest, CancelSplineBeforeExecuting) {
  SetEnabled(true);
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.25, 0.5, 0.5, 0.75, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.0, 0.25, 0.75, 1.0, 1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(1000));

  // Plan another spline, but don't start it yet:
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.75, 1.25, 1.5, 1.25, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.75, 1.25, 1.5, 1.75, 2.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(2);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  // Now execute it.
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(2);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  WaitForTrajectoryExecution();
  VerifyNearSplineGoal();
  VerifyNearPosition(1.0, 2.0, 5e-2);
}

// Tests that splines can excecute and plan at the same time.
TEST_F(DrivetrainTest, ParallelSplines) {
  SetEnabled(true);
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.25, 0.5, 0.5, 0.75, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.0, 0.25, 0.75, 1.0, 1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  // Second spline goal
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({1.0, 1.25, 1.5, 1.5, 1.25, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({1.0, 1.0, 1.25, 1.5, 1.75, 2.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(2);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(1);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  WaitForTrajectoryExecution();

  // Second start goal
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(2);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::milliseconds(4000));
  VerifyNearSplineGoal();
}

// Tests that a trajectory never told to execute will not replan.
TEST_F(DrivetrainTest, OnlyPlanSpline) {
  SetEnabled(true);
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.25, 0.5, 0.5, 0.75, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.0, 0.25, 0.75, 1.0, 1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  for (int i = 0; i < 100; ++i) {
    RunFor(dt());
    drivetrain_status_fetcher_.Fetch();
  }
  VerifyNearSplineGoal();
}

// Tests that a trajectory can be executed after it is planned.
TEST_F(DrivetrainTest, SplineExecuteAfterPlan) {
  SetEnabled(true);
  {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.25, 0.5, 0.5, 0.75, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.0, 0.25, 0.75, 1.0, 1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(1);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  RunFor(chrono::milliseconds(2000));

  // Start goal
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(1);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  WaitForTrajectoryExecution();

  VerifyNearPosition(1.0, 1.0, 5e-2);
}

// Tests that when we send a bunch of splines we properly evict old splines from
// the internal buffers.
TEST_F(DrivetrainTest, FillSplineBuffer) {
  SetEnabled(true);
  std::vector<int> expected_splines;
  constexpr size_t kExtraSplines = 10;
  constexpr size_t kNumStoredSplines = DrivetrainLoop::kNumSplineFetchers - 1;
  constexpr int kRunSpline = 1;
  {
    // Tell the drivetrain to execute spline 1; we then will check that that
    // spline never gets evicted.
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::SPLINE_FOLLOWER);
    goal_builder.add_spline_handle(kRunSpline);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  for (size_t spline_index = 0;
       spline_index < DrivetrainLoop::kNumSplineFetchers + kExtraSplines;
       ++spline_index) {
    auto builder = trajectory_goal_sender_.MakeBuilder();

    flatbuffers::Offset<flatbuffers::Vector<float>> spline_x_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.25, 0.5, 0.5, 0.75, 1.0});
    flatbuffers::Offset<flatbuffers::Vector<float>> spline_y_offset =
        builder.fbb()->CreateVector<float>({0.0, 0.0, 0.25, 0.75, 1.0, 1.0});

    MultiSpline::Builder multispline_builder =
        builder.MakeBuilder<MultiSpline>();

    multispline_builder.add_spline_count(1);
    multispline_builder.add_spline_x(spline_x_offset);
    multispline_builder.add_spline_y(spline_y_offset);

    flatbuffers::Offset<MultiSpline> multispline_offset =
        multispline_builder.Finish();

    SplineGoal::Builder spline_goal_builder = builder.MakeBuilder<SplineGoal>();
    spline_goal_builder.add_spline_idx(spline_index);
    spline_goal_builder.add_drive_spline_backwards(false);
    spline_goal_builder.add_spline(multispline_offset);
    ASSERT_EQ(builder.Send(spline_goal_builder.Finish()),
              aos::RawSender::Error::kOk);
    // Run for at least 2 iterations. Because of how the logic works, there will
    // actually typically be a single iteration where we store kNumStoredSplines
    // + 1.
    RunFor(2 * dt());

    expected_splines.push_back(spline_index);
    if (expected_splines.size() > kNumStoredSplines) {
      if (expected_splines.front() != kRunSpline) {
        expected_splines.erase(expected_splines.begin());
      } else {
        expected_splines.erase(expected_splines.begin() + 1);
      }
    }

    // We should always just have the past kNumStoredSplines available.
    drivetrain_status_fetcher_.Fetch();

    ASSERT_EQ(expected_splines.size(),
              CHECK_NOTNULL(drivetrain_status_fetcher_.get()
                                ->trajectory_logging()
                                ->available_splines())
                  ->size());
    for (size_t ii = 0; ii < expected_splines.size(); ++ii) {
      EXPECT_EQ(expected_splines[ii],
                CHECK_NOTNULL(drivetrain_status_fetcher_.get()
                                  ->trajectory_logging()
                                  ->available_splines())
                    ->Get(ii));
    }
  }
}

// The LineFollowDrivetrain logic is tested in line_follow_drivetrain_test. This
// tests that the integration with drivetrain_lib worked properly.
TEST_F(DrivetrainTest, BasicLineFollow) {
  SetEnabled(true);
  localizer_.target_selector()->set_has_target(true);
  localizer_.target_selector()->set_pose({{1.0, 1.0, 0.0}, M_PI_4});
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::LINE_FOLLOWER);
    goal_builder.add_throttle(0.5);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(5));

  drivetrain_status_fetcher_.Fetch();
  EXPECT_TRUE(CHECK_NOTNULL(drivetrain_status_fetcher_->line_follow_logging())
                  ->frozen());
  EXPECT_TRUE(CHECK_NOTNULL(drivetrain_status_fetcher_->line_follow_logging())
                  ->have_target());
  EXPECT_EQ(
      1.0,
      CHECK_NOTNULL(drivetrain_status_fetcher_->line_follow_logging())->x());
  EXPECT_EQ(
      1.0,
      CHECK_NOTNULL(drivetrain_status_fetcher_->line_follow_logging())->y());
  EXPECT_FLOAT_EQ(
      M_PI_4, CHECK_NOTNULL(drivetrain_status_fetcher_->line_follow_logging())
                  ->theta());

  // Should have run off the end of the target, running along the y=x line.
  EXPECT_LT(1.0, drivetrain_plant_.GetPosition().x());
  EXPECT_NEAR(drivetrain_plant_.GetPosition().x(),
              drivetrain_plant_.GetPosition().y(), 0.1);
}

// Tests that the line follower will not run and defer to regular open-loop
// driving when there is no target yet:
TEST_F(DrivetrainTest, LineFollowDefersToOpenLoop) {
  SetEnabled(true);
  localizer_.target_selector()->set_has_target(false);
  localizer_.target_selector()->set_pose({{1.0, 1.0, 0.0}, M_PI_4});
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::LINE_FOLLOWER);
    goal_builder.add_throttle(0.5);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(5));
  // Should have run straight (because we just set throttle, with wheel = 0)
  // along X-axis.
  EXPECT_LT(1.0, drivetrain_plant_.GetPosition().x());
  EXPECT_NEAR(0.0, drivetrain_plant_.GetPosition().y(), 1e-4);
}

// Tests that we can reset the localizer to a new position.
TEST_F(DrivetrainTest, ResetLocalizer) {
  SetEnabled(true);
  EXPECT_EQ(0.0, localizer_.x());
  EXPECT_EQ(0.0, localizer_.y());
  EXPECT_EQ(0.0, localizer_.theta());
  {
    auto builder = localizer_control_sender_.MakeBuilder();
    LocalizerControl::Builder localizer_control_builder =
        builder.MakeBuilder<LocalizerControl>();
    localizer_control_builder.add_x(9.0);
    localizer_control_builder.add_y(7.0);
    localizer_control_builder.add_theta(1.0);
    ASSERT_EQ(builder.Send(localizer_control_builder.Finish()),
              aos::RawSender::Error::kOk);
  }
  RunFor(dt());

  EXPECT_EQ(9.0, localizer_.x());
  EXPECT_EQ(7.0, localizer_.y());
  EXPECT_EQ(1.0, localizer_.theta());
}

// Tests that if wpilib_interface restarts, the drivetrain handles it.
TEST_F(DrivetrainTest, ResetDrivetrain) {
  SetEnabled(true);
  EXPECT_EQ(0.0, localizer_.x());
  EXPECT_EQ(0.0, localizer_.y());
  EXPECT_EQ(0.0, localizer_.theta());

  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    Goal::Builder goal_builder = builder.MakeBuilder<Goal>();
    goal_builder.add_controller_type(ControllerType::MOTION_PROFILE);
    goal_builder.add_left_goal(4.0);
    goal_builder.add_right_goal(4.0);
    ASSERT_EQ(builder.Send(goal_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

  RunFor(chrono::seconds(2));

  const double x_pos = localizer_.x();
  // The dead-reckoned X position in these sorts of situations tends to be
  // relatively poor with the current localizer, since it ignores voltage inputs
  // and models the robot's speed as always decaying towards zero, hence the
  // large tolerances on the x position.
  EXPECT_NEAR(4.0, x_pos, 2.0);
  EXPECT_NEAR(0.0, localizer_.y(), 1e-5);
  EXPECT_NEAR(0.0, localizer_.theta(), 1e-5);
  EXPECT_NEAR(4.0, localizer_.left_encoder(), 1e-3);
  EXPECT_NEAR(4.0, localizer_.right_encoder(), 1e-3);

  SimulateSensorReset();
  drivetrain_plant_.Reinitialize();

  RunFor(dt());

  EXPECT_EQ(x_pos, localizer_.x());
  EXPECT_NEAR(0.0, localizer_.y(), 1e-5);
  EXPECT_NEAR(0.0, localizer_.theta(), 1e-5);
  EXPECT_NEAR(0.0, localizer_.left_encoder(), 1e-5);
  EXPECT_NEAR(0.0, localizer_.right_encoder(), 1e-5);
}

// TODO(austin): Make sure the profile reset code when we disable works.

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
