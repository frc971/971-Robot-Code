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
#if defined(SUPPORT_PLOT)
#include "third_party/matplotlib-cpp/matplotlibcpp.h"
#endif

#include "frc971/control_loops/coerce_goal.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"
#include "frc971/control_loops/drivetrain/localizer.q.h"
#include "frc971/queues/gyro.q.h"

DEFINE_bool(plot, false, "If true, plot");

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

class DrivetrainTest : public ::aos::testing::ControlLoopTest {
 protected:
  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  ::frc971::control_loops::DrivetrainQueue my_drivetrain_queue_;

  ::aos::ShmEventLoop event_loop_;
  const DrivetrainConfig<double> dt_config_;
  DeadReckonEkf localizer_;
  // Create a loop and simulation plant.
  DrivetrainLoop drivetrain_motor_;
  ::aos::ShmEventLoop simulation_event_loop_;
  DrivetrainSimulation drivetrain_motor_plant_;

  DrivetrainTest()
      : my_drivetrain_queue_(".frc971.control_loops.drivetrain_queue",
                             ".frc971.control_loops.drivetrain_queue.goal",
                             ".frc971.control_loops.drivetrain_queue.position",
                             ".frc971.control_loops.drivetrain_queue.output",
                             ".frc971.control_loops.drivetrain_queue.status"),
        dt_config_(GetTestDrivetrainConfig()),
        localizer_(dt_config_),
        drivetrain_motor_(dt_config_, &event_loop_, &localizer_),
        drivetrain_motor_plant_(&simulation_event_loop_, dt_config_) {
    ::frc971::sensors::gyro_reading.Clear();
    set_battery_voltage(12.0);
  }

  void RunIteration() {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    SimulateTimestep(true, dt_config_.dt);
    if (FLAGS_plot) {
      my_drivetrain_queue_.status.FetchLatest();

      ::Eigen::Matrix<double, 2, 1> actual_position =
          drivetrain_motor_plant_.GetPosition();
      actual_x_.push_back(actual_position(0));
      actual_y_.push_back(actual_position(1));

      trajectory_x_.push_back(my_drivetrain_queue_.status->trajectory_logging.x);
      trajectory_y_.push_back(my_drivetrain_queue_.status->trajectory_logging.y);
    }
  }

  void RunForTime(monotonic_clock::duration run_for) {
    const auto end_time = monotonic_clock::now() + run_for;
    while (monotonic_clock::now() < end_time) {
      RunIteration();
    }
  }

  void TearDown() {
#if defined(SUPPORT_PLOT)
    if (FLAGS_plot) {
      std::cout << "plotting.\n";
      matplotlibcpp::figure();
      matplotlibcpp::plot(actual_x_, actual_y_, {{"label", "actual position"}});
      matplotlibcpp::plot(trajectory_x_, trajectory_y_,
                          {{"label", "trajectory position"}});
      matplotlibcpp::legend();
      matplotlibcpp::show();
    }
#endif
  }

  void VerifyNearGoal() {
    my_drivetrain_queue_.goal.FetchLatest();
    my_drivetrain_queue_.position.FetchLatest();
    EXPECT_NEAR(my_drivetrain_queue_.goal->left_goal,
                drivetrain_motor_plant_.GetLeftPosition(), 1e-3);
    EXPECT_NEAR(my_drivetrain_queue_.goal->right_goal,
                drivetrain_motor_plant_.GetRightPosition(), 1e-3);
  }

  void VerifyNearPosition(double x, double y) {
    my_drivetrain_queue_.status.FetchLatest();
    auto actual = drivetrain_motor_plant_.GetPosition();
    EXPECT_NEAR(actual(0), x, 1e-2);
    EXPECT_NEAR(actual(1), y, 1e-2);
  }

  void VerifyNearSplineGoal() {
    my_drivetrain_queue_.status.FetchLatest();
    double expected_x = my_drivetrain_queue_.status->trajectory_logging.x;
    double expected_y = my_drivetrain_queue_.status->trajectory_logging.y;
    auto actual = drivetrain_motor_plant_.GetPosition();
    EXPECT_NEAR(actual(0), expected_x, 2e-2);
    EXPECT_NEAR(actual(1), expected_y, 2e-2);
  }

  void WaitForTrajectoryPlan() {
    do {
      // Run for fewer iterations while the worker thread computes.
      ::std::this_thread::sleep_for(::std::chrono::milliseconds(5));
      RunIteration();
      my_drivetrain_queue_.status.FetchLatest();
    } while (my_drivetrain_queue_.status->trajectory_logging.planning_state !=
        (int8_t)SplineDrivetrain::PlanState::kPlannedTrajectory);
  }

  void WaitForTrajectoryExecution() {
    do {
      RunIteration();
      my_drivetrain_queue_.status.FetchLatest();
    } while (!my_drivetrain_queue_.status->trajectory_logging.is_executed);
  }

  virtual ~DrivetrainTest() { ::frc971::sensors::gyro_reading.Clear(); }

 private:
  ::std::vector<double> actual_x_;
  ::std::vector<double> actual_y_;
  ::std::vector<double> trajectory_x_;
  ::std::vector<double> trajectory_y_;

};

// Tests that the drivetrain converges on a goal.
TEST_F(DrivetrainTest, ConvergesCorrectly) {
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .controller_type(1)
      .left_goal(-1.0)
      .right_goal(1.0)
      .Send();
  RunForTime(chrono::seconds(2));
  VerifyNearGoal();
}

// Tests that the drivetrain converges on a goal when under the effect of a
// voltage offset/disturbance.
TEST_F(DrivetrainTest, ConvergesWithVoltageError) {
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .controller_type(1)
      .left_goal(-1.0)
      .right_goal(1.0)
      .Send();
  drivetrain_motor_plant_.set_left_voltage_offset(1.0);
  drivetrain_motor_plant_.set_right_voltage_offset(1.0);
  RunForTime(chrono::milliseconds(1500));
  VerifyNearGoal();
}

// Tests that it survives disabling.
TEST_F(DrivetrainTest, SurvivesDisabling) {
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .controller_type(1)
      .left_goal(-1.0)
      .right_goal(1.0)
      .Send();
  for (int i = 0; i < 500; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    if (i > 20 && i < 200) {
      SimulateTimestep(false, dt_config_.dt);
    } else {
      SimulateTimestep(true, dt_config_.dt);
    }
  }
  VerifyNearGoal();
}

// Tests that never having a goal doesn't break.
TEST_F(DrivetrainTest, NoGoalStart) {
  for (int i = 0; i < 20; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
  }
}

// Tests that never having a goal, but having driver's station messages, doesn't
// break.
TEST_F(DrivetrainTest, NoGoalWithRobotState) {
  RunForTime(chrono::milliseconds(100));
}

// Tests that the robot successfully drives straight forward.
// This used to not work due to a U-capping bug.
TEST_F(DrivetrainTest, DriveStraightForward) {
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .controller_type(1)
      .left_goal(4.0)
      .right_goal(4.0)
      .Send();
  for (int i = 0; i < 500; ++i) {
    RunIteration();
    ASSERT_TRUE(my_drivetrain_queue_.output.FetchLatest());
    EXPECT_NEAR(my_drivetrain_queue_.output->left_voltage,
                my_drivetrain_queue_.output->right_voltage, 1e-4);
    EXPECT_GT(my_drivetrain_queue_.output->left_voltage, -11);
    EXPECT_GT(my_drivetrain_queue_.output->right_voltage, -11);
  }
  VerifyNearGoal();
}

// Tests that the robot successfully drives close to straight.
// This used to fail in simulation due to libcdd issues with U-capping.
TEST_F(DrivetrainTest, DriveAlmostStraightForward) {
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .controller_type(1)
      .left_goal(4.0)
      .right_goal(3.9)
      .Send();
  for (int i = 0; i < 500; ++i) {
    RunIteration();
    ASSERT_TRUE(my_drivetrain_queue_.output.FetchLatest());
    EXPECT_GT(my_drivetrain_queue_.output->left_voltage, -11);
    EXPECT_GT(my_drivetrain_queue_.output->right_voltage, -11);
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
  {
    ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
        goal = my_drivetrain_queue_.goal.MakeMessage();
    goal->controller_type = 1;
    goal->left_goal = 4.0;
    goal->right_goal = 4.0;
    goal->linear.max_velocity = 1.0;
    goal->linear.max_acceleration = 3.0;
    goal->angular.max_velocity = 1.0;
    goal->angular.max_acceleration = 3.0;
    goal.Send();
  }

  while (monotonic_clock::now() <
         monotonic_clock::time_point(chrono::seconds(6))) {
    RunIteration();
    ASSERT_TRUE(my_drivetrain_queue_.output.FetchLatest());
    EXPECT_NEAR(my_drivetrain_queue_.output->left_voltage,
                my_drivetrain_queue_.output->right_voltage, 1e-4);
    EXPECT_GT(my_drivetrain_queue_.output->left_voltage, -6);
    EXPECT_GT(my_drivetrain_queue_.output->right_voltage, -6);
    EXPECT_LT(my_drivetrain_queue_.output->left_voltage, 6);
    EXPECT_LT(my_drivetrain_queue_.output->right_voltage, 6);
  }
  VerifyNearGoal();
}

// Tests that an angular motion profile succeeds.
TEST_F(DrivetrainTest, ProfileTurn) {
  {
    ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
        goal = my_drivetrain_queue_.goal.MakeMessage();
    goal->controller_type = 1;
    goal->left_goal = -1.0;
    goal->right_goal = 1.0;
    goal->linear.max_velocity = 1.0;
    goal->linear.max_acceleration = 3.0;
    goal->angular.max_velocity = 1.0;
    goal->angular.max_acceleration = 3.0;
    goal.Send();
  }

  while (monotonic_clock::now() <
         monotonic_clock::time_point(chrono::seconds(6))) {
    RunIteration();
    ASSERT_TRUE(my_drivetrain_queue_.output.FetchLatest());
    EXPECT_NEAR(my_drivetrain_queue_.output->left_voltage,
                -my_drivetrain_queue_.output->right_voltage, 1e-4);
    EXPECT_GT(my_drivetrain_queue_.output->left_voltage, -6);
    EXPECT_GT(my_drivetrain_queue_.output->right_voltage, -6);
    EXPECT_LT(my_drivetrain_queue_.output->left_voltage, 6);
    EXPECT_LT(my_drivetrain_queue_.output->right_voltage, 6);
  }
  VerifyNearGoal();
}

// Tests that a mixed turn drive saturated profile succeeds.
TEST_F(DrivetrainTest, SaturatedTurnDrive) {
  {
    ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
        goal = my_drivetrain_queue_.goal.MakeMessage();
    goal->controller_type = 1;
    goal->left_goal = 5.0;
    goal->right_goal = 4.0;
    goal->linear.max_velocity = 6.0;
    goal->linear.max_acceleration = 4.0;
    goal->angular.max_velocity = 2.0;
    goal->angular.max_acceleration = 4.0;
    goal.Send();
  }

  while (monotonic_clock::now() <
         monotonic_clock::time_point(chrono::seconds(3))) {
    RunIteration();
    ASSERT_TRUE(my_drivetrain_queue_.output.FetchLatest());
  }
  VerifyNearGoal();
}

// Tests that being in teleop drive for a bit and then transitioning to closed
// drive profiles nicely.
TEST_F(DrivetrainTest, OpenLoopThenClosed) {
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .controller_type(0)
      .wheel(0.0)
      .throttle(1.0)
      .highgear(true)
      .quickturn(false)
      .Send();

  RunForTime(chrono::seconds(1));

  my_drivetrain_queue_.goal.MakeWithBuilder()
      .controller_type(0)
      .wheel(0.0)
      .throttle(-0.3)
      .highgear(true)
      .quickturn(false)
      .Send();

  RunForTime(chrono::seconds(1));

  my_drivetrain_queue_.goal.MakeWithBuilder()
      .controller_type(0)
      .wheel(0.0)
      .throttle(0.0)
      .highgear(true)
      .quickturn(false)
      .Send();

  RunForTime(chrono::seconds(10));

  {
    ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
        goal = my_drivetrain_queue_.goal.MakeMessage();
    goal->controller_type = 1;
    goal->left_goal = 5.0;
    goal->right_goal = 4.0;
    goal->linear.max_velocity = 1.0;
    goal->linear.max_acceleration = 2.0;
    goal->angular.max_velocity = 1.0;
    goal->angular.max_acceleration = 2.0;
    goal.Send();
  }

  const auto end_time = monotonic_clock::now() + chrono::seconds(4);
  while (monotonic_clock::now() < end_time) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    drivetrain_motor_plant_.Simulate();
    SimulateTimestep(true, dt_config_.dt);
    ASSERT_TRUE(my_drivetrain_queue_.output.FetchLatest());
    EXPECT_GT(my_drivetrain_queue_.output->left_voltage, -6);
    EXPECT_GT(my_drivetrain_queue_.output->right_voltage, -6);
    EXPECT_LT(my_drivetrain_queue_.output->left_voltage, 6);
    EXPECT_LT(my_drivetrain_queue_.output->right_voltage, 6);
  }
  VerifyNearGoal();
}

// Tests that simple spline converges on a goal.
TEST_F(DrivetrainTest, SplineSimple) {
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 2;
  goal->spline.spline_idx = 1;
  goal->spline.spline_count = 1;
  goal->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
  goal->spline.spline_y = {{0.0, 0.0, 0.25 ,0.75, 1.0, 1.0}};
  goal.Send();
  RunIteration();

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
      start_goal = my_drivetrain_queue_.goal.MakeMessage();
  start_goal->controller_type = 2;
  start_goal->spline_handle = 1;
  start_goal.Send();
  WaitForTrajectoryPlan();

  RunForTime(chrono::milliseconds(2000));
  VerifyNearSplineGoal();
}

// Tests that we can drive a spline backwards.
TEST_F(DrivetrainTest, SplineSimpleBackwards) {
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 2;
  goal->spline.spline_idx = 1;
  goal->spline.spline_count = 1;
  goal->spline.drive_spline_backwards = true;
  goal->spline.spline_x = {{0.0, -0.25, -0.5, -0.5, -0.75, -1.0}};
  goal->spline.spline_y = {{0.0, 0.0, -0.25 ,-0.75, -1.0, -1.0}};
  goal.Send();
  RunIteration();

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
      start_goal = my_drivetrain_queue_.goal.MakeMessage();
  start_goal->controller_type = 2;
  start_goal->spline_handle = 1;
  start_goal.Send();
  WaitForTrajectoryPlan();

  // Check that we are right on the spline at the start (otherwise the feedback
  // will tend to correct for going the wrong direction).
  for (int ii = 0; ii < 10; ++ii) {
    RunForTime(chrono::milliseconds(100));
    VerifyNearSplineGoal();
  }

  WaitForTrajectoryExecution();

  VerifyNearSplineGoal();
  // Check that we are pointed the right direction:
  my_drivetrain_queue_.status.FetchLatest();
  auto actual = drivetrain_motor_plant_.state();
  const double expected_theta =
      my_drivetrain_queue_.status->trajectory_logging.theta;
  // As a sanity check, compare both against absolute angle and the spline's
  // goal angle.
  EXPECT_NEAR(0.0, ::aos::math::DiffAngle(actual(2), 0.0), 2e-2);
  EXPECT_NEAR(0.0, ::aos::math::DiffAngle(actual(2), expected_theta),
              2e-2);
}

// Tests that simple spline with a single goal message.
TEST_F(DrivetrainTest, SplineSingleGoal) {
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 2;
  goal->spline.spline_idx = 1;
  goal->spline.spline_count = 1;
  goal->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
  goal->spline.spline_y = {{0.0, 0.0, 0.25 ,0.75, 1.0, 1.0}};
  goal->spline_handle = 1;
  goal.Send();
  WaitForTrajectoryPlan();

  RunForTime(chrono::milliseconds(2000));
  VerifyNearSplineGoal();
}

// Tests that a trajectory can be stopped in the middle.
TEST_F(DrivetrainTest, SplineStop) {
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 2;
  goal->spline.spline_idx = 1;
  goal->spline.spline_count = 1;
  goal->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
  goal->spline.spline_y = {{0.0, 0.0, 0.25 ,0.75, 1.0, 1.0}};
  goal->spline_handle = 1;
  goal.Send();
  WaitForTrajectoryPlan();

  RunForTime(chrono::milliseconds(500));
  my_drivetrain_queue_.status.FetchLatest();
  const double goal_x = my_drivetrain_queue_.status->trajectory_logging.x;
  const double goal_y = my_drivetrain_queue_.status->trajectory_logging.y;

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> stop_goal =
      my_drivetrain_queue_.goal.MakeMessage();
  stop_goal->controller_type = 2;
  stop_goal->spline_handle = 0;
  stop_goal.Send();
  RunForTime(chrono::milliseconds(2000));

  // The goal shouldn't change after being stopped.
  my_drivetrain_queue_.status.FetchLatest();
  EXPECT_NEAR(my_drivetrain_queue_.status->trajectory_logging.x, goal_x, 1e-9);
  EXPECT_NEAR(my_drivetrain_queue_.status->trajectory_logging.y, goal_y, 1e-9);
}

// Tests that a spline can't be restarted.
TEST_F(DrivetrainTest, SplineRestart) {
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 2;
  goal->spline.spline_idx = 1;
  goal->spline.spline_count = 1;
  goal->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
  goal->spline.spline_y = {{0.0, 0.0, 0.25 ,0.75, 1.0, 1.0}};
  goal->spline_handle = 1;
  goal.Send();
  WaitForTrajectoryPlan();

  RunForTime(chrono::milliseconds(500));
  my_drivetrain_queue_.status.FetchLatest();
  const double goal_x = my_drivetrain_queue_.status->trajectory_logging.x;
  const double goal_y = my_drivetrain_queue_.status->trajectory_logging.y;

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> stop_goal =
      my_drivetrain_queue_.goal.MakeMessage();
  stop_goal->controller_type = 2;
  stop_goal->spline_handle = 0;
  stop_goal.Send();
  RunForTime(chrono::milliseconds(500));

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> restart_goal =
      my_drivetrain_queue_.goal.MakeMessage();
  restart_goal->controller_type = 2;
  restart_goal->spline_handle = 1;
  restart_goal.Send();
  RunForTime(chrono::milliseconds(2000));

  // The goal shouldn't change after being stopped and restarted.
  my_drivetrain_queue_.status.FetchLatest();
  EXPECT_NEAR(my_drivetrain_queue_.status->trajectory_logging.x, goal_x, 1e-9);
  EXPECT_NEAR(my_drivetrain_queue_.status->trajectory_logging.y, goal_y, 1e-9);
}

// Tests that simple spline converges when it doesn't start where it thinks.
TEST_F(DrivetrainTest, SplineOffset) {
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 2;
  goal->spline.spline_idx = 1;
  goal->spline.spline_count = 1;
  goal->spline.spline_x = {{0.2, 0.25, 0.5, 0.5, 0.75, 1.0}};
  goal->spline.spline_y = {{0.2, 0.0, 0.25 ,0.75, 1.0, 1.0}};
  goal.Send();
  RunIteration();

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
      start_goal = my_drivetrain_queue_.goal.MakeMessage();
  start_goal->controller_type = 2;
  start_goal->spline_handle = 1;
  start_goal.Send();
  WaitForTrajectoryPlan();

  RunForTime(chrono::milliseconds(2000));
  VerifyNearSplineGoal();
}

// Tests that simple spline converges when it starts to the side of where it
// thinks.
TEST_F(DrivetrainTest, SplineSideOffset) {
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 2;
  goal->spline.spline_idx = 1;
  goal->spline.spline_count = 1;
  goal->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
  goal->spline.spline_y = {{0.5, 0.0, 0.25 ,0.75, 1.0, 1.0}};
  goal.Send();
  RunIteration();

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
      start_goal = my_drivetrain_queue_.goal.MakeMessage();
  start_goal->controller_type = 2;
  start_goal->spline_handle = 1;
  start_goal.Send();
  WaitForTrajectoryPlan();

  RunForTime(chrono::milliseconds(2000));
  VerifyNearSplineGoal();
}

// Tests that a multispline converges on a goal.
TEST_F(DrivetrainTest, MultiSpline) {
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 2;
  goal->spline.spline_idx = 1;
  goal->spline.spline_count = 2;
  goal->spline.spline_x = {
      {0.0, 0.25, 0.5, 0.5, 0.75, 1.0, 1.25, 1.5, 1.5, 1.25, 1.0}};
  goal->spline.spline_y = {
      {0.0, 0.0, 0.25, 0.75, 1.0, 1.0, 1.0, 1.25, 1.5, 1.75, 2.0}};
  goal.Send();
  RunIteration();

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
      start_goal = my_drivetrain_queue_.goal.MakeMessage();
  start_goal->controller_type = 2;
  start_goal->spline_handle = 1;
  start_goal.Send();
  WaitForTrajectoryPlan();

  RunForTime(chrono::milliseconds(4000));
  VerifyNearSplineGoal();
}

// Tests that several splines converges on a goal.
TEST_F(DrivetrainTest, SequentialSplines) {
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 2;
  goal->spline.spline_idx = 1;
  goal->spline.spline_count = 1;
  goal->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
  goal->spline.spline_y = {{0.0, 0.0, 0.25, 0.75, 1.0, 1.0}};
  goal.Send();
  RunIteration();

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
      start_goal = my_drivetrain_queue_.goal.MakeMessage();
  start_goal->controller_type = 2;
  start_goal->spline_handle = 1;
  start_goal.Send();
  WaitForTrajectoryPlan();

  WaitForTrajectoryExecution();

  VerifyNearSplineGoal();

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
      second_spline_goal = my_drivetrain_queue_.goal.MakeMessage();
  second_spline_goal->controller_type = 2;
  second_spline_goal->spline.spline_idx = 2;
  second_spline_goal->spline.spline_count = 1;
  second_spline_goal->spline.spline_x = {{1.0, 1.25, 1.5, 1.5, 1.25, 1.0}};
  second_spline_goal->spline.spline_y = {{1.0, 1.0, 1.25, 1.5, 1.75, 2.0}};
  second_spline_goal.Send();
  RunIteration();

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
      second_start_goal = my_drivetrain_queue_.goal.MakeMessage();
  second_start_goal->controller_type = 2;
  second_start_goal->spline_handle = 2;
  second_start_goal.Send();
  WaitForTrajectoryPlan();

  RunForTime(chrono::milliseconds(2000));
  VerifyNearSplineGoal();
}
// Tests that a second spline will run if the first is stopped.
TEST_F(DrivetrainTest, SplineStopFirst) {
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 2;
  goal->spline.spline_idx = 1;
  goal->spline.spline_count = 1;
  goal->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
  goal->spline.spline_y = {{0.0, 0.0, 0.25 ,0.75, 1.0, 1.0}};
  goal->spline_handle = 1;
  goal.Send();
  WaitForTrajectoryPlan();

  RunForTime(chrono::milliseconds(1000));

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> stop_goal =
      my_drivetrain_queue_.goal.MakeMessage();
  stop_goal->controller_type = 2;
  stop_goal->spline_handle = 0;
  stop_goal.Send();
  RunForTime(chrono::milliseconds(500));

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
      second_spline_goal = my_drivetrain_queue_.goal.MakeMessage();
  second_spline_goal->controller_type = 2;
  second_spline_goal->spline.spline_idx = 2;
  second_spline_goal->spline.spline_count = 1;
  second_spline_goal->spline.spline_x = {{1.0, 1.25, 1.5, 1.5, 1.25, 1.0}};
  second_spline_goal->spline.spline_y = {{1.0, 1.0, 1.25, 1.5, 1.75, 2.0}};
  second_spline_goal->spline_handle = 2;
  second_spline_goal.Send();
  WaitForTrajectoryPlan();

  WaitForTrajectoryExecution();
  VerifyNearSplineGoal();
}

// Tests that we can run a second spline after having planned but never executed
// the first spline.
TEST_F(DrivetrainTest, CancelSplineBeforeExecuting) {
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 2;
  goal->spline.spline_idx = 1;
  goal->spline.spline_count = 1;
  goal->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
  goal->spline.spline_y = {{0.0, 0.0, 0.25 ,0.75, 1.0, 1.0}};
  // Don't start running the splane.
  goal->spline_handle = 0;
  goal.Send();
  WaitForTrajectoryPlan();

  RunForTime(chrono::milliseconds(1000));

  // Plan another spline, but don't start it yet:
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
      second_spline_goal = my_drivetrain_queue_.goal.MakeMessage();
  second_spline_goal->controller_type = 2;
  second_spline_goal->spline.spline_idx = 2;
  second_spline_goal->spline.spline_count = 1;
  second_spline_goal->spline.spline_x = {{0.0, 0.75, 1.25, 1.5, 1.25, 1.0}};
  second_spline_goal->spline.spline_y = {{0.0, 0.75, 1.25, 1.5, 1.75, 2.0}};
  second_spline_goal->spline_handle = 0;
  second_spline_goal.Send();
  WaitForTrajectoryPlan();

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
      execute_goal = my_drivetrain_queue_.goal.MakeMessage();
  execute_goal->controller_type = 2;
  execute_goal->spline_handle = 2;
  execute_goal.Send();

  WaitForTrajectoryExecution();
  VerifyNearSplineGoal();
  VerifyNearPosition(1.0, 2.0);
}

// Tests that splines can excecute and plan at the same time.
TEST_F(DrivetrainTest, ParallelSplines) {
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 2;
  goal->spline.spline_idx = 1;
  goal->spline.spline_count = 1;
  goal->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
  goal->spline.spline_y = {{0.0, 0.0, 0.25, 0.75, 1.0, 1.0}};
  goal.Send();
  WaitForTrajectoryPlan();

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
      second_spline_goal = my_drivetrain_queue_.goal.MakeMessage();
  second_spline_goal->spline_handle = 1;
  second_spline_goal->controller_type = 2;
  second_spline_goal->spline.spline_idx = 2;
  second_spline_goal->spline.spline_count = 1;
  second_spline_goal->spline.spline_x = {{1.0, 1.25, 1.5, 1.5, 1.25, 1.0}};
  second_spline_goal->spline.spline_y = {{1.0, 1.0, 1.25, 1.5, 1.75, 2.0}};
  second_spline_goal.Send();
  WaitForTrajectoryExecution();

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
      second_start_goal = my_drivetrain_queue_.goal.MakeMessage();
  second_start_goal->controller_type = 2;
  second_start_goal->spline_handle = 2;
  second_start_goal.Send();

  RunForTime(chrono::milliseconds(4000));
  VerifyNearSplineGoal();
}

//Tests that a trajectory never told to execute will not replan.
TEST_F(DrivetrainTest, OnlyPlanSpline) {
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 2;
  goal->spline.spline_idx = 1;
  goal->spline.spline_count = 1;
  goal->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
  goal->spline.spline_y = {{0.0, 0.0, 0.25, 0.75, 1.0, 1.0}};
  goal.Send();
  WaitForTrajectoryPlan();

  for (int i = 0; i < 100; ++i) {
    RunIteration();
    my_drivetrain_queue_.status.FetchLatest();
    EXPECT_EQ(my_drivetrain_queue_.status->trajectory_logging.planning_state, 3);
    ::std::this_thread::sleep_for(::std::chrono::milliseconds(2));
  }
  VerifyNearSplineGoal();
}

//Tests that a trajectory can be executed after it is planned.
TEST_F(DrivetrainTest, SplineExecuteAfterPlan) {
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 2;
  goal->spline.spline_idx = 1;
  goal->spline.spline_count = 1;
  goal->spline.spline_x = {{0.0, 0.25, 0.5, 0.5, 0.75, 1.0}};
  goal->spline.spline_y = {{0.0, 0.0, 0.25, 0.75, 1.0, 1.0}};
  goal.Send();
  WaitForTrajectoryPlan();
  RunForTime(chrono::milliseconds(2000));

  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal>
      start_goal = my_drivetrain_queue_.goal.MakeMessage();
  start_goal->controller_type = 2;
  start_goal->spline_handle = 1;
  start_goal.Send();
  RunForTime(chrono::milliseconds(2000));

  VerifyNearPosition(1.0, 1.0);
}

// The LineFollowDrivetrain logic is tested in line_follow_drivetrain_test. This
// tests that the integration with drivetrain_lib worked properly.
TEST_F(DrivetrainTest, BasicLineFollow) {
  localizer_.target_selector()->set_has_target(true);
  localizer_.target_selector()->set_pose({{1.0, 1.0, 0.0}, M_PI_4});
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 3;
  goal->throttle = 0.5;
  goal.Send();

  RunForTime(chrono::seconds(5));

  my_drivetrain_queue_.status.FetchLatest();
  EXPECT_TRUE(my_drivetrain_queue_.status->line_follow_logging.frozen);
  EXPECT_TRUE(my_drivetrain_queue_.status->line_follow_logging.have_target);
  EXPECT_EQ(1.0, my_drivetrain_queue_.status->line_follow_logging.x);
  EXPECT_EQ(1.0, my_drivetrain_queue_.status->line_follow_logging.y);
  EXPECT_FLOAT_EQ(M_PI_4,
                  my_drivetrain_queue_.status->line_follow_logging.theta);

  // Should have run off the end of the target, running along the y=x line.
  EXPECT_LT(1.0, drivetrain_motor_plant_.GetPosition().x());
  EXPECT_NEAR(drivetrain_motor_plant_.GetPosition().x(),
              drivetrain_motor_plant_.GetPosition().y(), 0.1);
}

// Tests that the line follower will not run and defer to regular open-loop
// driving when there is no target yet:
TEST_F(DrivetrainTest, LineFollowDefersToOpenLoop) {
  localizer_.target_selector()->set_has_target(false);
  localizer_.target_selector()->set_pose({{1.0, 1.0, 0.0}, M_PI_4});
  ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Goal> goal =
      my_drivetrain_queue_.goal.MakeMessage();
  goal->controller_type = 3;
  goal->throttle = 0.5;
  goal.Send();

  RunForTime(chrono::seconds(5));
  // Should have run straight (because we just set throttle, with wheel = 0)
  // along X-axis.
  EXPECT_LT(1.0, drivetrain_motor_plant_.GetPosition().x());
  EXPECT_NEAR(0.0, drivetrain_motor_plant_.GetPosition().y(), 1e-4);
}

// Tests that we can reset the localizer to a new position.
TEST_F(DrivetrainTest, ResetLocalizer) {
  EXPECT_EQ(0.0, localizer_.x());
  EXPECT_EQ(0.0, localizer_.y());
  EXPECT_EQ(0.0, localizer_.theta());
  ::aos::Queue<LocalizerControl> localizer_queue(
      ".frc971.control_loops.drivetrain.localizer_control");
  ASSERT_TRUE(
      localizer_queue.MakeWithBuilder().x(9.0).y(7.0).theta(1.0).Send());
  RunIteration();

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
