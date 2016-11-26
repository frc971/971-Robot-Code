#include <stdio.h>

#include <memory>

#include "aos/common/util/phased_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"

#include "y2015_bot3/autonomous/auto.q.h"
#include "y2015_bot3/actors/drivetrain_actor.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "y2015_bot3/control_loops/elevator/elevator.q.h"
#include "y2015_bot3/control_loops/intake/intake.q.h"
#include "y2015_bot3/control_loops/drivetrain/drivetrain_base.h"

using ::aos::time::Time;
using ::frc971::control_loops::drivetrain_queue;
using ::y2015_bot3::control_loops::intake_queue;
using ::y2015_bot3::control_loops::elevator_queue;

namespace y2015_bot3 {
namespace autonomous {

struct ProfileParams {
  double velocity;
  double acceleration;
};

namespace time = ::aos::time;
namespace actors = ::frc971::actors;

const ProfileParams kFastDrive = {3.0, 3.5};
const ProfileParams kLastDrive = {4.0, 4.0};
const ProfileParams kStackingFirstTurn = {3.0, 7.0};
const ProfileParams kFinalDriveTurn = {3.0, 5.0};
const ProfileParams kSlowFirstDriveTurn = {0.75, 1.5};
const ProfileParams kSlowSecondDriveTurn = {0.6, 1.5};
const ProfileParams kCanPickupDrive = {1.3, 3.0};

static double left_initial_position, right_initial_position;

bool ShouldExitAuto() {
  ::y2015_bot3::autonomous::autonomous.FetchLatest();
  bool ans = !::y2015_bot3::autonomous::autonomous->run_auto;
  if (ans) {
    LOG(INFO, "Time to exit auto mode\n");
  }
  return ans;
}

void ResetDrivetrain() {
  LOG(INFO, "resetting the drivetrain\n");
  drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(left_initial_position)
      .left_velocity_goal(0)
      .right_goal(right_initial_position)
      .right_velocity_goal(0)
      .Send();
}

void InitializeEncoders() {
  drivetrain_queue.status.FetchAnother();
  left_initial_position = drivetrain_queue.status->estimated_left_position;
  right_initial_position = drivetrain_queue.status->estimated_right_position;
}

::std::unique_ptr< ::frc971::actors::DrivetrainAction> SetDriveGoal(
    double distance, const ProfileParams drive_params, double theta,
    const ProfileParams &turn_params) {
  LOG(INFO, "Driving to %f\n", distance);

  actors::DrivetrainActionParams params;
  params.left_initial_position = left_initial_position;
  params.right_initial_position = right_initial_position;
  params.y_offset = distance;
  params.theta_offset = theta;
  params.maximum_turn_acceleration = turn_params.acceleration;
  params.maximum_turn_velocity = turn_params.velocity;
  params.maximum_velocity = drive_params.velocity;
  params.maximum_acceleration = drive_params.acceleration;
  auto drivetrain_action = actors::MakeDrivetrainAction(params);

  drivetrain_action->Start();
  left_initial_position +=
      distance - theta * control_loops::drivetrain::kDrivetrainTurnWidth / 2.0;
  right_initial_position +=
      distance + theta * control_loops::drivetrain::kDrivetrainTurnWidth / 2.0;
  return ::std::move(drivetrain_action);
}
void WaitUntilDoneOrCanceled(
    ::std::unique_ptr<aos::common::actions::Action> action) {
  if (!action) {
    LOG(ERROR, "No action, not waiting\n");
    return;
  }
  while (true) {
    // Poll the running bit and auto done bits.
    ::aos::time::PhasedLoopXMS(5, 2500);
    if (!action->Running() || ShouldExitAuto()) {
      return;
    }
  }
}

void WaitUntilNear(double distance) {
  while (true) {
    if (ShouldExitAuto()) return;
    drivetrain_queue.status.FetchAnother();
    double left_error =
        ::std::abs(left_initial_position -
                   drivetrain_queue.status->estimated_left_position);
    double right_error =
        ::std::abs(right_initial_position -
                   drivetrain_queue.status->estimated_right_position);
    const double kPositionThreshold = 0.05 + distance;
    if (right_error < kPositionThreshold && left_error < kPositionThreshold) {
      LOG(INFO, "At the goal\n");
      return;
    }
  }
}

void GrabberForTime(double voltage, double wait_time) {
  ::aos::time::Time now = ::aos::time::Time::Now();
  ::aos::time::Time end_time = now + time::Time::InSeconds(wait_time);
  LOG(INFO, "Starting to grab at %f for %f seconds\n", voltage, wait_time);

  while (true) {
    autonomous::can_grabber_control.MakeWithBuilder()
        .can_grabber_voltage(voltage).can_grabbers(false).Send();

    // Poll the running bit and auto done bits.
    if (ShouldExitAuto()) {
      return;
    }
    if (::aos::time::Time::Now() > end_time) {
      LOG(INFO, "Done grabbing\n");
      return;
    }
    ::aos::time::PhasedLoopXMS(5, 2500);
  }
}

// Auto methods.
void CanGrabberAuto() {
  ResetDrivetrain();

  // Launch can grabbers.
  //GrabberForTime(12.0, 0.26);
  GrabberForTime(6.0, 0.40);
  if (ShouldExitAuto()) return;
  InitializeEncoders();
  ResetDrivetrain();
  if (ShouldExitAuto()) return;
  // Send our intake goals.
  if (!intake_queue.goal.MakeWithBuilder().movement(0.0).claw_closed(true)
          .Send()) {
    LOG(ERROR, "Sending intake goal failed.\n");
  }
  {
    auto new_elevator_goal = elevator_queue.goal.MakeMessage();
    new_elevator_goal->max_velocity = 0.0;
    new_elevator_goal->max_acceleration = 0.0;
    new_elevator_goal->height = 0.030;
    new_elevator_goal->velocity = 0.0;
    new_elevator_goal->passive_support = true;
    new_elevator_goal->can_support = false;

    if (new_elevator_goal.Send()) {
      LOG(DEBUG, "sending goals: elevator: %f\n", 0.03);
    } else {
      LOG(ERROR, "Sending elevator goal failed.\n");
    }
  }

  const double kMoveBackDistance = 1.75;
  //const double kMoveBackDistance = 0.0;

  // Drive backwards, and pulse the can grabbers again to tip the cans.
  drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(true)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(left_initial_position + kMoveBackDistance)
      .left_velocity_goal(0)
      .right_goal(right_initial_position + kMoveBackDistance)
      .right_velocity_goal(0)
      .Send();
  GrabberForTime(12.0, 0.02);
  if (ShouldExitAuto()) return;

  // We shouldn't need as much power at this point, so lower the can grabber
  // voltages to avoid damaging the motors due to stalling.
  GrabberForTime(4.0, 0.75);
  if (ShouldExitAuto()) return;
  GrabberForTime(-3.0, 0.25);
  if (ShouldExitAuto()) return;
  GrabberForTime(-12.0, 1.0);
  if (ShouldExitAuto()) return;
  GrabberForTime(-3.0, 12.0);
}

const ProfileParams kElevatorProfile = {1.0, 5.0};

static double elevator_goal_height = 0.0;

void SetElevatorHeight(double height, const ProfileParams params,
                       bool can_open = false, bool support_open = true) {
  // Send our elevator goals, with limits set in the profile params.
  auto new_elevator_goal = elevator_queue.goal.MakeMessage();
  elevator_goal_height = height;
  new_elevator_goal->max_velocity = params.velocity;
  new_elevator_goal->max_acceleration = params.acceleration;
  new_elevator_goal->height = elevator_goal_height;
  new_elevator_goal->velocity = 0.0;
  new_elevator_goal->passive_support = support_open;
  new_elevator_goal->can_support = can_open;

  if (new_elevator_goal.Send()) {
    LOG(DEBUG, "sending goals: elevator: %f\n", elevator_goal_height);
  } else {
    LOG(ERROR, "Sending elevator goal failed.\n");
  }
}

void WaitForElevator() {
  while (true) {
    if (ShouldExitAuto()) return;
    control_loops::elevator_queue.status.FetchAnother();

    constexpr double kProfileError = 1e-5;
    constexpr double kEpsilon = 0.03;

    if (::std::abs(control_loops::elevator_queue.status->goal_height -
                   elevator_goal_height) <
            kProfileError &&
        ::std::abs(control_loops::elevator_queue.status->goal_velocity) <
            kProfileError) {
      LOG(INFO, "Profile done.\n");
      if (::std::abs(control_loops::elevator_queue.status->height -
                     elevator_goal_height) <
          kEpsilon) {
        LOG(INFO, "Near goal, done.\n");
        return;
      }
    }
  }
}

void WaitUntilElevatorBelow(double height) {
  while (true) {
    if (ShouldExitAuto()) return;
    control_loops::elevator_queue.status.FetchAnother();

    if (control_loops::elevator_queue.status->goal_height < height) {
      LOG(INFO, "Profile below.\n");
      if (control_loops::elevator_queue.status->height < height) {
        LOG(INFO, "Elevator below goal, done.\n");
        return;
      }
    }
  }
}

void WaitUntilElevatorAbove(double height) {
  while (true) {
    if (ShouldExitAuto()) return;
    control_loops::elevator_queue.status.FetchAnother();

    if (control_loops::elevator_queue.status->goal_height > height) {
      LOG(INFO, "Profile above.\n");
      if (control_loops::elevator_queue.status->height > height) {
        LOG(INFO, "Elevator above goal, done.\n");
        return;
      }
    }
  }
}

void LiftTote(double final_height = 0.48) {
  // Send our intake goals.
  if (!intake_queue.goal.MakeWithBuilder().movement(10.0).claw_closed(true)
          .Send()) {
    LOG(ERROR, "Sending intake goal failed.\n");
  }

  while (true) {
    elevator_queue.status.FetchAnother();
    if (!elevator_queue.status.get()) {
      LOG(ERROR, "Got no elevator status packet.\n");
    }
    if (ShouldExitAuto()) return;
    if (elevator_queue.status->has_tote) {
      break;
    }
  }
  if (!intake_queue.goal.MakeWithBuilder().movement(0.0).claw_closed(true)
          .Send()) {
    LOG(ERROR, "Sending intake goal failed.\n");
  }
  SetElevatorHeight(0.02, kElevatorProfile, false, false);
  WaitUntilElevatorBelow(0.05);
  if (ShouldExitAuto()) return;

  SetElevatorHeight(final_height, kElevatorProfile, false, false);
}

void TripleCanAuto() {
  ::aos::time::Time start_time = ::aos::time::Time::Now();

  ::std::unique_ptr<::frc971::actors::DrivetrainAction> drive;
  InitializeEncoders();
  ResetDrivetrain();

  SetElevatorHeight(0.03, kElevatorProfile, false, true);


  LiftTote();
  if (ShouldExitAuto()) return;

  // The amount to turn out for going around the first can.
  const double kFirstTurn = 0.5;

  drive = SetDriveGoal(0.0, kFastDrive, kFirstTurn, kStackingFirstTurn);

  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  drive = SetDriveGoal(2.0, kFastDrive, -kFirstTurn * 2.0, kSlowFirstDriveTurn);

  WaitUntilNear(1.5);
  if (ShouldExitAuto()) return;

  if (!intake_queue.goal.MakeWithBuilder().movement(0.0).claw_closed(false)
          .Send()) {
    LOG(ERROR, "Sending intake goal failed.\n");
  }

  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  drive = SetDriveGoal(0.0, kFastDrive, kFirstTurn, kStackingFirstTurn);
  LiftTote();
  if (ShouldExitAuto()) return;
  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  // The amount to turn out for going around the second can.
  const double kSecondTurn = 0.35;
  const double kSecondTurnExtraOnSecond = 0.10;

  drive = SetDriveGoal(0.0, kFastDrive, kSecondTurn, kStackingFirstTurn);
  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  drive =
      SetDriveGoal(2.05, kFastDrive, -kSecondTurn * 2.0 - kSecondTurnExtraOnSecond, kSlowSecondDriveTurn);
  WaitUntilNear(1.5);

  if (!intake_queue.goal.MakeWithBuilder().movement(0.0).claw_closed(false)
          .Send()) {
    LOG(ERROR, "Sending intake goal failed.\n");
  }

  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  drive = SetDriveGoal(0.0, kFastDrive, kSecondTurn + kSecondTurnExtraOnSecond, kStackingFirstTurn);

  LiftTote(0.18);

  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  drive = SetDriveGoal(0.3, kFastDrive, -1.7, kFinalDriveTurn);
  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  if (!intake_queue.goal.MakeWithBuilder().movement(0.0).claw_closed(false)
          .Send()) {
    LOG(ERROR, "Sending intake goal failed.\n");
  }

  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;
  drive = SetDriveGoal(2.95, kLastDrive, 0.0, kFinalDriveTurn);

  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  SetElevatorHeight(0.03, kElevatorProfile, true, true);
  WaitForElevator();
  if (ShouldExitAuto()) return;

  drive = SetDriveGoal(-2.25, kFastDrive, 0.0, kFinalDriveTurn);
  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  drive = SetDriveGoal(0.0, kFastDrive, 1.80, kFinalDriveTurn);
  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  if (!intake_queue.goal.MakeWithBuilder().movement(10.0).claw_closed(true)
          .Send()) {
    LOG(ERROR, "Sending intake goal failed.\n");
  }
  SetElevatorHeight(0.03, kElevatorProfile, false, true);

  drive = SetDriveGoal(1.0, kCanPickupDrive, 0.0, kFinalDriveTurn);
  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  SetElevatorHeight(0.03, kElevatorProfile, true, true);

  while (true) {
    elevator_queue.status.FetchAnother();
    if (!elevator_queue.status.get()) {
      LOG(ERROR, "Got no elevator status packet.\n");
    }
    if (ShouldExitAuto()) return;
    if (elevator_queue.status->has_tote) {
      LOG(INFO, "Got the tote!\n");
      break;
    }
    if ((::aos::time::Time::Now() - start_time) > time::Time::InSeconds(15.0)) {
      LOG(INFO, "Out of time\n");
      break;
    }
  }

  ::aos::time::Time end_time = ::aos::time::Time::Now();
  LOG(INFO, "Ended auto with %f to spare\n",
      (time::Time::InSeconds(15.0) - (end_time - start_time)).ToSeconds());
  if (!intake_queue.goal.MakeWithBuilder().movement(0.0).claw_closed(true)
          .Send()) {
    LOG(ERROR, "Sending intake goal failed.\n");
  }
}

void HandleAuto() {
  ::aos::time::Time start_time = ::aos::time::Time::Now();
  LOG(INFO, "Starting auto mode at %f\n", start_time.ToSeconds());

  // TODO(comran): Add various options for different autos down below.
  //CanGrabberAuto();
  TripleCanAuto();
}

}  // namespace autonomous
}  // namespace y2015_bot3
