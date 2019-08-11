#include <unistd.h>

#include <chrono>
#include <memory>

#include "aos/controls/control_loop_test.h"
#include "aos/controls/polytope.h"
#include "aos/events/event-loop.h"
#include "aos/events/shm-event-loop.h"
#include "aos/time/time.h"
#include "gflags/gflags.h"
#include "gtest/gtest.h"

#include "frc971/control_loops/coerce_goal.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"
#include "frc971/control_loops/drivetrain/localizer.q.h"
#include "frc971/queues/gyro.q.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

class DrivetrainTest : public ::aos::testing::ControlLoopTest {
 protected:
  DrivetrainTest()
      : ::aos::testing::ControlLoopTest(GetTestDrivetrainConfig().dt),
        test_event_loop_(MakeEventLoop()),
        drivetrain_goal_sender_(
            test_event_loop_
                ->MakeSender<::frc971::control_loops::DrivetrainQueue::Goal>(
                    ".frc971.control_loops.drivetrain_queue.goal")),
        drivetrain_goal_fetcher_(
            test_event_loop_
                ->MakeFetcher<::frc971::control_loops::DrivetrainQueue::Goal>(
                    ".frc971.control_loops.drivetrain_queue.goal")),
        drivetrain_status_fetcher_(
            test_event_loop_
                ->MakeFetcher<::frc971::control_loops::DrivetrainQueue::Status>(
                    ".frc971.control_loops.drivetrain_queue.status")),
        drivetrain_output_fetcher_(
            test_event_loop_
                ->MakeFetcher<::frc971::control_loops::DrivetrainQueue::Output>(
                    ".frc971.control_loops.drivetrain_queue.output")),
        localizer_control_sender_(
            test_event_loop_->MakeSender<LocalizerControl>(
                ".frc971.control_loops.drivetrain.localizer_control")),
        drivetrain_event_loop_(MakeEventLoop()),
        dt_config_(GetTestDrivetrainConfig()),
        localizer_(drivetrain_event_loop_.get(), dt_config_),
        drivetrain_(dt_config_, drivetrain_event_loop_.get(), &localizer_),
        drivetrain_plant_event_loop_(MakeEventLoop()),
        drivetrain_plant_(drivetrain_plant_event_loop_.get(), dt_config_) {
    set_battery_voltage(12.0);
  }
  virtual ~DrivetrainTest() {}

  void TearDown() { drivetrain_plant_.MaybePlot(); }

  void VerifyNearGoal() {
    drivetrain_goal_fetcher_.Fetch();
    EXPECT_NEAR(drivetrain_goal_fetcher_->left_goal,
                drivetrain_plant_.GetLeftPosition(), 1e-3);
    EXPECT_NEAR(drivetrain_goal_fetcher_->right_goal,
                drivetrain_plant_.GetRightPosition(), 1e-3);
  }

  void VerifyNearPosition(double x, double y) {
    auto actual = drivetrain_plant_.GetPosition();
    EXPECT_NEAR(actual(0), x, 1e-2);
    EXPECT_NEAR(actual(1), y, 1e-2);
  }

  void VerifyNearSplineGoal() {
    drivetrain_status_fetcher_.Fetch();
    const double expected_x = drivetrain_status_fetcher_->trajectory_logging.x;
    const double expected_y = drivetrain_status_fetcher_->trajectory_logging.y;
    const ::Eigen::Vector2d actual = drivetrain_plant_.GetPosition();
    EXPECT_NEAR(actual(0), expected_x, 2e-2);
    EXPECT_NEAR(actual(1), expected_y, 2e-2);
  }

  void WaitForTrajectoryPlan() {
    do {
      // Run for fewer iterations while the worker thread computes.
      ::std::this_thread::sleep_for(::std::chrono::milliseconds(5));
      RunFor(dt());
      EXPECT_TRUE(drivetrain_status_fetcher_.Fetch());
    } while (drivetrain_status_fetcher_->trajectory_logging.planning_state !=
             (int8_t)SplineDrivetrain::PlanState::kPlannedTrajectory);
  }

  void WaitForTrajectoryExecution() {
    do {
      RunFor(dt());
      EXPECT_TRUE(drivetrain_status_fetcher_.Fetch());
    } while (!drivetrain_status_fetcher_->trajectory_logging.is_executed);
  }

  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::Sender<::frc971::control_loops::DrivetrainQueue::Goal>
      drivetrain_goal_sender_;
  ::aos::Fetcher<::frc971::control_loops::DrivetrainQueue::Goal>
      drivetrain_goal_fetcher_;
  ::aos::Fetcher<::frc971::control_loops::DrivetrainQueue::Status>
      drivetrain_status_fetcher_;
  ::aos::Fetcher<::frc971::control_loops::DrivetrainQueue::Output>
      drivetrain_output_fetcher_;
  ::aos::Sender<LocalizerControl> localizer_control_sender_;

  ::std::unique_ptr<::aos::EventLoop> drivetrain_event_loop_;
  const DrivetrainConfig<double> dt_config_;
  DeadReckonEkf localizer_;
  // Create a loop and simulation plant.
  DrivetrainLoop drivetrain_;

  ::std::unique_ptr<::aos::EventLoop> drivetrain_plant_event_loop_;
  DrivetrainSimulation drivetrain_plant_;
};

// Tests that the drivetrain converges on a goal.
TEST_F(DrivetrainTest, ConvergesCorrectly) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 1;
    message->left_goal = -1.0;
    message->right_goal = 1.0;
    EXPECT_TRUE(message.Send());
  }
  RunFor(chrono::seconds(2));
  VerifyNearGoal();
}

// Tests that the drivetrain converges on a goal when under the effect of a
// voltage offset/disturbance.
TEST_F(DrivetrainTest, ConvergesWithVoltageError) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 1;
    message->left_goal = -1.0;
    message->right_goal = 1.0;
    EXPECT_TRUE(message.Send());
  }
  drivetrain_plant_.set_left_voltage_offset(1.0);
  drivetrain_plant_.set_right_voltage_offset(1.0);
  RunFor(chrono::milliseconds(1500));
  VerifyNearGoal();
}

// Tests that it survives disabling.
TEST_F(DrivetrainTest, SurvivesDisabling) {
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 1;
    message->left_goal = -1.0;
    message->right_goal = 1.0;
    EXPECT_TRUE(message.Send());
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
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 1;
    message->left_goal = 4.0;
    message->right_goal = 4.0;
    EXPECT_TRUE(message.Send());
  }

  for (int i = 0; i < 500; ++i) {
    RunFor(dt());
    ASSERT_TRUE(drivetrain_output_fetcher_.Fetch());
    EXPECT_NEAR(drivetrain_output_fetcher_->left_voltage,
                drivetrain_output_fetcher_->right_voltage, 1e-4);
    EXPECT_GT(drivetrain_output_fetcher_->left_voltage, -11);
    EXPECT_GT(drivetrain_output_fetcher_->right_voltage, -11);
  }
  VerifyNearGoal();
}

// Tests that the robot successfully drives close to straight.
// This used to fail in simulation due to libcdd issues with U-capping.
TEST_F(DrivetrainTest, DriveAlmostStraightForward) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 1;
    message->left_goal = 4.0;
    message->right_goal = 3.9;
    EXPECT_TRUE(message.Send());
  }
  for (int i = 0; i < 500; ++i) {
    RunFor(dt());
    ASSERT_TRUE(drivetrain_output_fetcher_.Fetch());
    EXPECT_GT(drivetrain_output_fetcher_->left_voltage, -11);
    EXPECT_GT(drivetrain_output_fetcher_->right_voltage, -11);
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
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 1;
    message->left_goal = 4.0;
    message->right_goal = 4.0;
    message->linear.max_velocity = 1.0;
    message->linear.max_acceleration = 3.0;
    message->angular.max_velocity = 1.0;
    message->angular.max_acceleration = 3.0;
    EXPECT_TRUE(message.Send());
  }

  while (monotonic_now() < monotonic_clock::time_point(chrono::seconds(6))) {
    RunFor(dt());
    ASSERT_TRUE(drivetrain_output_fetcher_.Fetch());
    EXPECT_NEAR(drivetrain_output_fetcher_->left_voltage,
                drivetrain_output_fetcher_->right_voltage, 1e-4);
    EXPECT_GT(drivetrain_output_fetcher_->left_voltage, -6);
    EXPECT_GT(drivetrain_output_fetcher_->right_voltage, -6);
    EXPECT_LT(drivetrain_output_fetcher_->left_voltage, 6);
    EXPECT_LT(drivetrain_output_fetcher_->right_voltage, 6);
  }
  VerifyNearGoal();
}

// Tests that an angular motion profile succeeds.
TEST_F(DrivetrainTest, ProfileTurn) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 1;
    message->left_goal = -1.0;
    message->right_goal = 1.0;
    message->linear.max_velocity = 1.0;
    message->linear.max_acceleration = 3.0;
    message->angular.max_velocity = 1.0;
    message->angular.max_acceleration = 3.0;
    EXPECT_TRUE(message.Send());
  }

  while (monotonic_now() < monotonic_clock::time_point(chrono::seconds(6))) {
    RunFor(dt());
    ASSERT_TRUE(drivetrain_output_fetcher_.Fetch());
    EXPECT_NEAR(drivetrain_output_fetcher_->left_voltage,
                -drivetrain_output_fetcher_->right_voltage, 1e-4);
    EXPECT_GT(drivetrain_output_fetcher_->left_voltage, -6);
    EXPECT_GT(drivetrain_output_fetcher_->right_voltage, -6);
    EXPECT_LT(drivetrain_output_fetcher_->left_voltage, 6);
    EXPECT_LT(drivetrain_output_fetcher_->right_voltage, 6);
  }
  VerifyNearGoal();
}

// Tests that a mixed turn drive saturated profile succeeds.
TEST_F(DrivetrainTest, SaturatedTurnDrive) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 1;
    message->left_goal = 5.0;
    message->right_goal = 4.0;
    message->linear.max_velocity = 6.0;
    message->linear.max_acceleration = 4.0;
    message->angular.max_velocity = 2.0;
    message->angular.max_acceleration = 4.0;
    EXPECT_TRUE(message.Send());
  }

  while (monotonic_now() < monotonic_clock::time_point(chrono::seconds(3))) {
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
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 0;
    message->wheel = 0.0;
    message->throttle = 1.0;
    message->highgear = true;
    message->quickturn = false;
    EXPECT_TRUE(message.Send());
  }

  RunFor(chrono::seconds(1));

  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 0;
    message->wheel = 0.0;
    message->throttle = -0.3;
    message->highgear = true;
    message->quickturn = false;
    EXPECT_TRUE(message.Send());
  }

  RunFor(chrono::seconds(1));

  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 0;
    message->wheel = 0.0;
    message->throttle = 0.0;
    message->highgear = true;
    message->quickturn = false;
    EXPECT_TRUE(message.Send());
  }

  RunFor(chrono::seconds(10));

  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 1;
    message->left_goal = 5.0;
    message->right_goal = 4.0;
    message->linear.max_velocity = 1.0;
    message->linear.max_acceleration = 2.0;
    message->angular.max_velocity = 1.0;
    message->angular.max_acceleration = 2.0;
    EXPECT_TRUE(message.Send());
  }

  const auto end_time = monotonic_now() + chrono::seconds(4);
  while (monotonic_now() < end_time) {
    RunFor(dt());
    ASSERT_TRUE(drivetrain_output_fetcher_.Fetch());
    EXPECT_GT(drivetrain_output_fetcher_->left_voltage, -6);
    EXPECT_GT(drivetrain_output_fetcher_->right_voltage, -6);
    EXPECT_LT(drivetrain_output_fetcher_->left_voltage, 6);
    EXPECT_LT(drivetrain_output_fetcher_->right_voltage, 6);
  }
  VerifyNearGoal();
}

// Tests that simple spline converges on a goal.
TEST_F(DrivetrainTest, SplineSimple) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 1;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
    message->spline.spline_y = {{0.0, 0.0, 0.25, 0.75, 1.0, 1.0}};
    EXPECT_TRUE(message.Send());
  }
  RunFor(dt());

  // Send the start goal
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline_handle = 1;
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  RunFor(chrono::milliseconds(2000));
  VerifyNearSplineGoal();
}

// Tests that we can drive a spline backwards.
TEST_F(DrivetrainTest, SplineSimpleBackwards) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 1;
    message->spline.spline_count = 1;
    message->spline.drive_spline_backwards = true;
    message->spline.spline_x = {{0.0, -0.25, -0.5, -0.5, -0.75, -1.0}};
    message->spline.spline_y = {{0.0, 0.0, -0.25, -0.75, -1.0, -1.0}};
    EXPECT_TRUE(message.Send());
  }
  RunFor(dt());

  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline_handle = 1;
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

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
      drivetrain_status_fetcher_->trajectory_logging.theta;
  // As a sanity check, compare both against absolute angle and the spline's
  // goal angle.
  EXPECT_NEAR(0.0, ::aos::math::DiffAngle(actual(2), 0.0), 2e-2);
  EXPECT_NEAR(0.0, ::aos::math::DiffAngle(actual(2), expected_theta), 2e-2);
}

// Tests that simple spline with a single goal message.
TEST_F(DrivetrainTest, SplineSingleGoal) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 1;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
    message->spline.spline_y = {{0.0, 0.0, 0.25, 0.75, 1.0, 1.0}};
    message->spline_handle = 1;
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  RunFor(chrono::milliseconds(2000));
  VerifyNearSplineGoal();
}

// Tests that a trajectory can be stopped in the middle.
TEST_F(DrivetrainTest, SplineStop) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 1;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
    message->spline.spline_y = {{0.0, 0.0, 0.25, 0.75, 1.0, 1.0}};
    message->spline_handle = 1;
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  RunFor(chrono::milliseconds(500));
  drivetrain_status_fetcher_.Fetch();
  const double goal_x = drivetrain_status_fetcher_->trajectory_logging.x;
  const double goal_y = drivetrain_status_fetcher_->trajectory_logging.y;

  // Now stop.
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline_handle = 0;
    EXPECT_TRUE(message.Send());
  }
  RunFor(chrono::milliseconds(2000));

  // The goal shouldn't change after being stopped.
  drivetrain_status_fetcher_.Fetch();
  EXPECT_NEAR(drivetrain_status_fetcher_->trajectory_logging.x, goal_x, 1e-9);
  EXPECT_NEAR(drivetrain_status_fetcher_->trajectory_logging.y, goal_y, 1e-9);
}

// Tests that a spline can't be restarted.
TEST_F(DrivetrainTest, SplineRestart) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 1;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
    message->spline.spline_y = {{0.0, 0.0, 0.25, 0.75, 1.0, 1.0}};
    message->spline_handle = 1;
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  RunFor(chrono::milliseconds(500));
  drivetrain_status_fetcher_.Fetch();
  const double goal_x = drivetrain_status_fetcher_->trajectory_logging.x;
  const double goal_y = drivetrain_status_fetcher_->trajectory_logging.y;

  // Send a stop goal.
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline_handle = 0;
    EXPECT_TRUE(message.Send());
  }
  RunFor(chrono::milliseconds(500));

  // Send a restart.
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline_handle = 1;
    EXPECT_TRUE(message.Send());
  }
  RunFor(chrono::milliseconds(2000));

  // The goal shouldn't change after being stopped and restarted.
  drivetrain_status_fetcher_.Fetch();
  EXPECT_NEAR(drivetrain_status_fetcher_->trajectory_logging.x, goal_x, 1e-9);
  EXPECT_NEAR(drivetrain_status_fetcher_->trajectory_logging.y, goal_y, 1e-9);
}

// Tests that simple spline converges when it doesn't start where it thinks.
TEST_F(DrivetrainTest, SplineOffset) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 1;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{0.2, 0.25, 0.5, 0.5, 0.75, 1.0}};
    message->spline.spline_y = {{0.2, 0.0, 0.25, 0.75, 1.0, 1.0}};
    EXPECT_TRUE(message.Send());
  }
  RunFor(dt());

  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline_handle = 1;
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  RunFor(chrono::milliseconds(2000));
  VerifyNearSplineGoal();
}

// Tests that simple spline converges when it starts to the side of where it
// thinks.
TEST_F(DrivetrainTest, SplineSideOffset) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 1;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
    message->spline.spline_y = {{0.5, 0.0, 0.25, 0.75, 1.0, 1.0}};
    EXPECT_TRUE(message.Send());
  }
  RunFor(dt());

  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline_handle = 1;
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  RunFor(chrono::milliseconds(2000));
  VerifyNearSplineGoal();
}

// Tests that a multispline converges on a goal.
TEST_F(DrivetrainTest, MultiSpline) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 1;
    message->spline.spline_count = 2;
    message->spline.spline_x = {
        {0.0, 0.25, 0.5, 0.5, 0.75, 1.0, 1.25, 1.5, 1.5, 1.25, 1.0}};
    message->spline.spline_y = {
        {0.0, 0.0, 0.25, 0.75, 1.0, 1.0, 1.0, 1.25, 1.5, 1.75, 2.0}};
    EXPECT_TRUE(message.Send());
  }
  RunFor(dt());

  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline_handle = 1;
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  RunFor(chrono::milliseconds(4000));
  VerifyNearSplineGoal();
}

// Tests that several splines converges on a goal.
TEST_F(DrivetrainTest, SequentialSplines) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 1;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
    message->spline.spline_y = {{0.0, 0.0, 0.25, 0.75, 1.0, 1.0}};
    EXPECT_TRUE(message.Send());
  }
  RunFor(dt());

  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline_handle = 1;
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  WaitForTrajectoryExecution();

  VerifyNearSplineGoal();

  // Second spline.
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 2;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{1.0, 1.25, 1.5, 1.5, 1.25, 1.0}};
    message->spline.spline_y = {{1.0, 1.0, 1.25, 1.5, 1.75, 2.0}};
    EXPECT_TRUE(message.Send());
  }
  RunFor(dt());

  // And then start it.
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline_handle = 2;
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  RunFor(chrono::milliseconds(2000));
  VerifyNearSplineGoal();
}
// Tests that a second spline will run if the first is stopped.
TEST_F(DrivetrainTest, SplineStopFirst) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 1;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
    message->spline.spline_y = {{0.0, 0.0, 0.25, 0.75, 1.0, 1.0}};
    message->spline_handle = 1;
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  RunFor(chrono::milliseconds(1000));

  // Stop goal
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline_handle = 0;
    EXPECT_TRUE(message.Send());
  }
  RunFor(chrono::milliseconds(500));

  // Second spline goal.
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 2;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{1.0, 1.25, 1.5, 1.5, 1.25, 1.0}};
    message->spline.spline_y = {{1.0, 1.0, 1.25, 1.5, 1.75, 2.0}};
    message->spline_handle = 2;
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  WaitForTrajectoryExecution();
  VerifyNearSplineGoal();
}

// Tests that we can run a second spline after having planned but never executed
// the first spline.
TEST_F(DrivetrainTest, CancelSplineBeforeExecuting) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 1;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
    message->spline.spline_y = {{0.0, 0.0, 0.25, 0.75, 1.0, 1.0}};
    // Don't start running the splane.
    message->spline_handle = 0;
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  RunFor(chrono::milliseconds(1000));

  // Plan another spline, but don't start it yet:
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 2;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{0.0, 0.75, 1.25, 1.5, 1.25, 1.0}};
    message->spline.spline_y = {{0.0, 0.75, 1.25, 1.5, 1.75, 2.0}};
    message->spline_handle = 0;
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  // Now execute it.
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline_handle = 2;
    EXPECT_TRUE(message.Send());
  }

  WaitForTrajectoryExecution();
  VerifyNearSplineGoal();
  VerifyNearPosition(1.0, 2.0);
}

// Tests that splines can excecute and plan at the same time.
TEST_F(DrivetrainTest, ParallelSplines) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 1;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
    message->spline.spline_y = {{0.0, 0.0, 0.25, 0.75, 1.0, 1.0}};
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  // Second spline goal
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->spline_handle = 1;
    message->controller_type = 2;
    message->spline.spline_idx = 2;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{1.0, 1.25, 1.5, 1.5, 1.25, 1.0}};
    message->spline.spline_y = {{1.0, 1.0, 1.25, 1.5, 1.75, 2.0}};
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryExecution();

  // Second start goal
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline_handle = 2;
    EXPECT_TRUE(message.Send());
  }

  RunFor(chrono::milliseconds(4000));
  VerifyNearSplineGoal();
}

// Tests that a trajectory never told to execute will not replan.
TEST_F(DrivetrainTest, OnlyPlanSpline) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 1;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
    message->spline.spline_y = {{0.0, 0.0, 0.25, 0.75, 1.0, 1.0}};
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();

  for (int i = 0; i < 100; ++i) {
    RunFor(dt());
    drivetrain_status_fetcher_.Fetch();
    EXPECT_EQ(drivetrain_status_fetcher_->trajectory_logging.planning_state,
              3);
    ::std::this_thread::sleep_for(::std::chrono::milliseconds(2));
  }
  VerifyNearSplineGoal();
}

// Tests that a trajectory can be executed after it is planned.
TEST_F(DrivetrainTest, SplineExecuteAfterPlan) {
  SetEnabled(true);
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline.spline_idx = 1;
    message->spline.spline_count = 1;
    message->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
    message->spline.spline_y = {{0.0, 0.0, 0.25, 0.75, 1.0, 1.0}};
    EXPECT_TRUE(message.Send());
  }
  WaitForTrajectoryPlan();
  RunFor(chrono::milliseconds(2000));

  // Start goal
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 2;
    message->spline_handle = 1;
    EXPECT_TRUE(message.Send());
  }
  RunFor(chrono::milliseconds(2000));

  VerifyNearPosition(1.0, 1.0);
}

// The LineFollowDrivetrain logic is tested in line_follow_drivetrain_test. This
// tests that the integration with drivetrain_lib worked properly.
TEST_F(DrivetrainTest, BasicLineFollow) {
  SetEnabled(true);
  localizer_.target_selector()->set_has_target(true);
  localizer_.target_selector()->set_pose({{1.0, 1.0, 0.0}, M_PI_4});
  {
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 3;
    message->throttle = 0.5;
    EXPECT_TRUE(message.Send());
  }

  RunFor(chrono::seconds(5));

  drivetrain_status_fetcher_.Fetch();
  EXPECT_TRUE(drivetrain_status_fetcher_->line_follow_logging.frozen);
  EXPECT_TRUE(drivetrain_status_fetcher_->line_follow_logging.have_target);
  EXPECT_EQ(1.0, drivetrain_status_fetcher_->line_follow_logging.x);
  EXPECT_EQ(1.0, drivetrain_status_fetcher_->line_follow_logging.y);
  EXPECT_FLOAT_EQ(M_PI_4,
                  drivetrain_status_fetcher_->line_follow_logging.theta);

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
    auto message = drivetrain_goal_sender_.MakeMessage();
    message->controller_type = 3;
    message->throttle = 0.5;
    EXPECT_TRUE(message.Send());
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
    auto message = localizer_control_sender_.MakeMessage();
    message->x = 9.0;
    message->y = 7.0;
    message->theta = 1.0;
    ASSERT_TRUE(message.Send());
  }
  RunFor(dt());

  EXPECT_EQ(9.0, localizer_.x());
  EXPECT_EQ(7.0, localizer_.y());
  EXPECT_EQ(1.0, localizer_.theta());
}

::aos::controls::HVPolytope<2, 4, 4> MakeBox(double x1_min, double x1_max,
                                             double x2_min, double x2_max) {
  Eigen::Matrix<double, 4, 2> box_H;
  box_H << /*[[*/ 1.0, 0.0 /*]*/,
      /*[*/ -1.0, 0.0 /*]*/,
      /*[*/ 0.0, 1.0 /*]*/,
      /*[*/ 0.0, -1.0 /*]]*/;
  Eigen::Matrix<double, 4, 1> box_k;
  box_k << /*[[*/ x1_max /*]*/,
      /*[*/ -x1_min /*]*/,
      /*[*/ x2_max /*]*/,
      /*[*/ -x2_min /*]]*/;
  ::aos::controls::HPolytope<2> t_poly(box_H, box_k);
  return ::aos::controls::HVPolytope<2, 4, 4>(t_poly.H(), t_poly.k(),
                                              t_poly.Vertices());
}

class CoerceGoalTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// WHOOOHH!
TEST_F(CoerceGoalTest, Inside) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << /*[[*/ 1, -1 /*]]*/;

  Eigen::Matrix<double, 2, 1> R;
  R << /*[[*/ 1.5, 1.5 /*]]*/;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal<double>(box, K, 0, R);

  EXPECT_EQ(R(0, 0), output(0, 0));
  EXPECT_EQ(R(1, 0), output(1, 0));
}

TEST_F(CoerceGoalTest, Outside_Inside_Intersect) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, -1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal<double>(box, K, 0, R);

  EXPECT_EQ(2.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, Outside_Inside_no_Intersect) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(3, 4, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, -1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal<double>(box, K, 0, R);

  EXPECT_EQ(3.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, Middle_Of_Edge) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(0, 4, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << -1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal<double>(box, K, 0, R);

  EXPECT_EQ(2.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, PerpendicularLine) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal<double>(box, K, 0, R);

  EXPECT_EQ(1.0, output(0, 0));
  EXPECT_EQ(1.0, output(1, 0));
}

// TODO(austin): Make sure the profile reset code when we disable works.

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
