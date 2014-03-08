#include <stdio.h>

#include <memory>

#include "aos/common/control_loop/Timing.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/logging/logging.h"
#include "aos/common/network/team_number.h"

#include "frc971/autonomous/auto.q.h"
#include "frc971/constants.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/shooter/shooter.q.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/actions/action_client.h"
#include "frc971/actions/shoot_action.h"
#include "frc971/actions/drivetrain_action.h"

using ::aos::time::Time;

namespace frc971 {
namespace autonomous {

namespace time = ::aos::time;

static double left_initial_position, right_initial_position;

bool ShouldExitAuto() {
  ::frc971::autonomous::autonomous.FetchLatest();
  bool ans = !::frc971::autonomous::autonomous->run_auto;
  if (ans) {
    LOG(INFO, "Time to exit auto mode\n");
  }
  return ans;
}

void StopDrivetrain() {
  LOG(INFO, "Stopping the drivetrain\n");
  control_loops::drivetrain.goal.MakeWithBuilder()
      .control_loop_driving(true)
      .left_goal(left_initial_position)
      .left_velocity_goal(0)
      .right_goal(right_initial_position)
      .right_velocity_goal(0)
      .quickturn(false)
      .Send();
}

void ResetDrivetrain() {
  LOG(INFO, "resetting the drivetrain\n");
  control_loops::drivetrain.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .highgear(false)
      .steering(0.0)
      .throttle(0.0)
      .Send();
}

void DriveSpin(double radians) {
  LOG(INFO, "going to spin %f\n", radians);

  ::aos::util::TrapezoidProfile profile(::aos::time::Time::InMS(10));
  ::Eigen::Matrix<double, 2, 1> driveTrainState;
  const double goal_velocity = 0.0;
  const double epsilon = 0.01;
  // in drivetrain "meters"
  const double kRobotWidth = 0.4544;

  profile.set_maximum_acceleration(1.5);
  profile.set_maximum_velocity(0.8);

  const double side_offset = kRobotWidth * radians / 2.0;

  while (true) {
    ::aos::time::PhasedLoop10MS(5000);      // wait until next 10ms tick
    driveTrainState = profile.Update(side_offset, goal_velocity);

    if (::std::abs(driveTrainState(0, 0) - side_offset) < epsilon) break;
    if (ShouldExitAuto()) return;

    LOG(DEBUG, "Driving left to %f, right to %f\n",
        left_initial_position - driveTrainState(0, 0),
        right_initial_position + driveTrainState(0, 0));
    control_loops::drivetrain.goal.MakeWithBuilder()
        .control_loop_driving(true)
        .highgear(false)
        .left_goal(left_initial_position - driveTrainState(0, 0))
        .right_goal(right_initial_position + driveTrainState(0, 0))
        .left_velocity_goal(-driveTrainState(1, 0))
        .right_velocity_goal(driveTrainState(1, 0))
        .Send();
  }
  left_initial_position -= side_offset;
  right_initial_position += side_offset;
  LOG(INFO, "Done moving\n");
}

void PositionClawVertically(double intake_power = 0.0, double centering_power = 0.0) {
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder()
           .bottom_angle(0.0)
           .separation_angle(0.0)
           .intake(intake_power)
           .centering(centering_power)
           .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
  }
}

void PositionClawBackIntake() {
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder()
           .bottom_angle(-2.273474)
           .separation_angle(0.0)
           .intake(12.0)
           .centering(12.0)
           .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
  }
}

void PositionClawForShot() {
  // Turn the claw on, keep it straight up until the ball has been grabbed.
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder()
           .bottom_angle(0.9)
           .separation_angle(0.1)
           .intake(3.0)
           .centering(1.0)
           .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
  }
}

void SetShotPower(double power) {
  if (!control_loops::shooter_queue_group.goal.MakeWithBuilder()
           .shot_power(power)
           .shot_requested(false)
           .unload_requested(false)
           .load_requested(false)
           .Send()) {
    LOG(WARNING, "sending shooter goal failed\n");
  }
}

void WaitUntilDoneOrCanceled(Action *action) {
  while (true) {
    // Poll the running bit and auto done bits.
    ::aos::time::PhasedLoop10MS(5000);
    if (!action->Running() || ShouldExitAuto()) {
      return;
    }
  }
}

void Shoot() {
  // Shoot.
  auto shoot_action = actions::MakeShootAction();
  shoot_action->Start();
  WaitUntilDoneOrCanceled(shoot_action.get());
}

::std::unique_ptr<TypedAction< ::frc971::actions::DrivetrainActionQueueGroup>>
SetDriveGoal(double distance, double maximum_velocity = 1.5) {
  auto drivetrain_action = actions::MakeDrivetrainAction();
  drivetrain_action->GetGoal()->left_initial_position = left_initial_position;
  drivetrain_action->GetGoal()->right_initial_position = right_initial_position;
  drivetrain_action->GetGoal()->y_offset = distance;
  drivetrain_action->GetGoal()->maximum_velocity = maximum_velocity;
  drivetrain_action->Start();
  left_initial_position += distance;
  right_initial_position += distance;
  return ::std::move(drivetrain_action);
}

void InitializeEncoders() {
  control_loops::drivetrain.position.FetchLatest();
  while (!control_loops::drivetrain.position.get()) {
    LOG(WARNING, "No previous drivetrain position packet, trying to fetch again\n");
    control_loops::drivetrain.position.FetchNextBlocking();
  }
  left_initial_position =
    control_loops::drivetrain.position->left_encoder;
  right_initial_position =
    control_loops::drivetrain.position->right_encoder;

}

void HandleAuto() {
  LOG(INFO, "Handling auto mode\n");
  ResetDrivetrain();

  if (ShouldExitAuto()) return;
  InitializeEncoders();

  // Turn the claw on, keep it straight up until the ball has been grabbed.
  PositionClawVertically(12.0, 4.0);
  SetShotPower(100.0);

  // Wait for the ball to enter the claw.
  time::SleepFor(time::Time::InSeconds(0.5));
  if (ShouldExitAuto()) return;
  PositionClawForShot();

  {
    if (ShouldExitAuto()) return;
    // Drive to the goal.
    auto drivetrain_action = SetDriveGoal(3.0);
    WaitUntilDoneOrCanceled(drivetrain_action.get());
    if (ShouldExitAuto()) return;
  }

  // Shoot.
  Shoot();
  time::SleepFor(time::Time::InSeconds(0.1));

  {
    if (ShouldExitAuto()) return;
    // Intake the new ball.
    PositionClawBackIntake();
    auto drivetrain_action = SetDriveGoal(-0.3);
    WaitUntilDoneOrCanceled(drivetrain_action.get());
    if (ShouldExitAuto()) return;
  }

  // Drive back.
  {
    auto drivetrain_action = SetDriveGoal(3.0);
    time::SleepFor(time::Time::InSeconds(0.5));
    if (ShouldExitAuto()) return;
    PositionClawForShot();
    WaitUntilDoneOrCanceled(drivetrain_action.get());
    if (ShouldExitAuto()) return;
  }

  // Shoot
  Shoot();
  if (ShouldExitAuto()) return;

  // Get ready to zero when we come back up.
  PositionClawVertically(0.0, 0.0);
}

}  // namespace autonomous
}  // namespace frc971
