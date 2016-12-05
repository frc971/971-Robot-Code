#include <stdio.h>

#include <memory>

#include "aos/common/util/phased_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"

#include "frc971/autonomous/auto.q.h"
#include "y2015/autonomous/auto.q.h"
#include "y2015/constants.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2015/actors/drivetrain_actor.h"
#include "y2015/control_loops/claw/claw.q.h"
#include "y2015/control_loops/fridge/fridge.q.h"
#include "y2015/actors/pickup_actor.h"
#include "y2015/actors/stack_actor.h"
#include "y2015/actors/held_to_lift_actor.h"

using ::aos::time::Time;
using ::frc971::control_loops::drivetrain_queue;
using ::y2015::control_loops::claw_queue;
using ::y2015::control_loops::fridge::fridge_queue;

namespace y2015 {
namespace autonomous {

constexpr double kClawAutoVelocity = 3.00;
constexpr double kClawAutoAcceleration = 6.0;
constexpr double kAngleEpsilon = 0.10;
const double kClawTotePackAngle = 0.90;
const double kArmRaiseLowerClearance = -0.08;
const double kClawStackClearance = 0.55;
const double kStackUpHeight = 0.60;
const double kStackUpArm = 0.0;

struct ProfileParams {
  double velocity;
  double acceleration;
};

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
  drivetrain_queue.goal.MakeWithBuilder()
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
  drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(false)
      //.highgear(false)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(left_initial_position)
      .left_velocity_goal(0)
      .right_goal(right_initial_position)
      .right_velocity_goal(0)
      .Send();
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

void StepDrive(double distance, double theta) {
  double left_goal = (left_initial_position + distance -
                      theta * constants::GetValues().turn_width / 2.0);
  double right_goal = (right_initial_position + distance +
                       theta * constants::GetValues().turn_width / 2.0);
  drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(true)
      .left_goal(left_goal)
      .right_goal(right_goal)
      .left_velocity_goal(0.0)
      .right_velocity_goal(0.0)
      .Send();
  left_initial_position = left_goal;
  right_initial_position = right_goal;
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

const ProfileParams kFastDrive = {2.0, 3.5};
const ProfileParams kFastKnockDrive = {2.0, 3.0};
const ProfileParams kStackingSecondDrive = {2.0, 1.5};
const ProfileParams kFastTurn = {3.0, 10.0};
const ProfileParams kStackingFirstTurn = {1.0, 1.0};
const ProfileParams kStackingSecondTurn = {2.0, 6.0};
const ProfileParams kComboTurn = {1.2, 8.0};
const ProfileParams kRaceTurn = {4.0, 10.0};
const ProfileParams kRaceDrive = {2.0, 2.0};
const ProfileParams kRaceBackupDrive = {2.0, 5.0};

::std::unique_ptr<::y2015::actors::DrivetrainAction> SetDriveGoal(
    double distance, const ProfileParams drive_params, double theta = 0,
    const ProfileParams &turn_params = kFastTurn) {
  LOG(INFO, "Driving to %f\n", distance);

  ::y2015::actors::DrivetrainActionParams params;
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
      distance - theta * constants::GetValues().turn_width / 2.0;
  right_initial_position +=
      distance + theta * constants::GetValues().turn_width / 2.0;
  return ::std::move(drivetrain_action);
}

const ProfileParams kFridgeYProfile{1.0, 4.0};
const ProfileParams kFridgeXProfile{1.0, 2.0};
const ProfileParams kFridgeFastXProfile{1.2, 5.0};

static double fridge_goal_x = 0.0;
static double fridge_goal_y = 0.0;

void MoveFridge(double x, double y, bool grabbers, const ProfileParams x_params,
                const ProfileParams y_params) {
  auto new_fridge_goal = fridge_queue.goal.MakeMessage();
  new_fridge_goal->profiling_type = 1;

  new_fridge_goal->max_y_velocity = y_params.velocity;
  new_fridge_goal->max_y_acceleration = y_params.acceleration;
  new_fridge_goal->y = y;
  fridge_goal_y = y;
  new_fridge_goal->y_velocity = 0.0;

  new_fridge_goal->max_x_velocity = x_params.velocity;
  new_fridge_goal->max_x_acceleration = x_params.acceleration;
  new_fridge_goal->x = x;
  fridge_goal_x = x;
  new_fridge_goal->x_velocity = 0.0;

  new_fridge_goal->grabbers.top_front = grabbers;
  new_fridge_goal->grabbers.top_back = grabbers;
  new_fridge_goal->grabbers.bottom_front = grabbers;
  new_fridge_goal->grabbers.bottom_back = grabbers;

  if (!new_fridge_goal.Send()) {
    LOG(ERROR, "Sending fridge goal failed.\n");
    return;
  }
}

void WaitForFridge() {
  while (true) {
    if (ShouldExitAuto()) return;
    fridge_queue.status.FetchAnother();

    constexpr double kProfileError = 1e-5;
    constexpr double kXEpsilon = 0.03, kYEpsilon = 0.03;

    if (fridge_queue.status->state != 4) {
      LOG(ERROR, "Fridge no longer running, aborting action\n");
      return;
    }

    if (::std::abs(fridge_queue.status->goal_x - fridge_goal_x) <
            kProfileError &&
        ::std::abs(fridge_queue.status->goal_y - fridge_goal_y) <
            kProfileError &&
        ::std::abs(fridge_queue.status->goal_x_velocity) < kProfileError &&
        ::std::abs(fridge_queue.status->goal_y_velocity) < kProfileError) {
      LOG(INFO, "Profile done.\n");
      if (::std::abs(fridge_queue.status->x - fridge_goal_x) <
              kXEpsilon &&
          ::std::abs(fridge_queue.status->y - fridge_goal_y) <
              kYEpsilon) {
        LOG(INFO, "Near goal, done.\n");
        return;
      }
    }
  }
}

void InitializeEncoders() {
  drivetrain_queue.status.FetchAnother();
  left_initial_position = drivetrain_queue.status->estimated_left_position;
  right_initial_position = drivetrain_queue.status->estimated_right_position;
}

void WaitForClawZero() {
  LOG(INFO, "Waiting for claw to zero.\n");
  while (true) {
    control_loops::claw_queue.status.FetchAnother();
    LOG_STRUCT(DEBUG, "Got claw status", *control_loops::claw_queue.status);
    if (control_loops::claw_queue.status->zeroed) {
      LOG(INFO, "Claw zeroed\n");
      return;
    }

    if (ShouldExitAuto()) return;
  }
}

void WaitForFridgeZero() {
  LOG(INFO, "Waiting for claw to zero.\n");
  while (true) {
    fridge_queue.status.FetchAnother();
    LOG_STRUCT(DEBUG, "Got fridge status", *fridge_queue.status);
    if (fridge_queue.status->zeroed) {
      LOG(INFO, "Fridge zeroed\n");
      return;
    }

    if (ShouldExitAuto()) return;
  }
}

constexpr ProfileParams kDefaultClawParams = {kClawAutoVelocity,
                                              kClawAutoAcceleration};

// Move the claw in a very small number of cycles.
constexpr ProfileParams kInstantaneousClaw = {100.0, 100.0};
constexpr ProfileParams kFastClaw = {8.0, 20.0};

void SetClawStateNoWait(double angle, double intake_voltage,
                        double rollers_closed,
                        const ProfileParams &claw_params = kDefaultClawParams) {
  auto message = control_loops::claw_queue.goal.MakeMessage();
  message->angle = angle;
  message->max_velocity = claw_params.velocity;
  message->max_acceleration = claw_params.acceleration;
  message->angular_velocity = 0.0;
  message->intake = intake_voltage;
  message->rollers_closed = rollers_closed;

  LOG_STRUCT(DEBUG, "Sending claw goal", *message);
  message.Send();
}

void SetClawState(double angle, double intake_voltage, double rollers_closed,
                  const ProfileParams &claw_params = kDefaultClawParams) {
  SetClawStateNoWait(angle, intake_voltage, rollers_closed, claw_params);
  while (true) {
    control_loops::claw_queue.status.FetchAnother();
    const double current_angle = control_loops::claw_queue.status->angle;
    LOG_STRUCT(DEBUG, "Got claw status", *control_loops::claw_queue.status);

    // I believe all we can care about here is the angle. Other values will
    // either be set or not, but there is nothing we can do about it. If it
    // never gets there we do not care, auto is over for us.
    if (::std::abs(current_angle - angle) < kAngleEpsilon) {
      break;
    }
    if (ShouldExitAuto()) return;
  }
}

void TripleCanAuto() {
  ::std::unique_ptr<::y2015::actors::DrivetrainAction> drive;
  ::std::unique_ptr<::y2015::actors::PickupAction> pickup;
  ::std::unique_ptr<::y2015::actors::StackAction> stack;
  ::std::unique_ptr<::y2015::actors::HeldToLiftAction> lift;

  actors::PickupParams pickup_params;
  // Lift to here initially.
  pickup_params.pickup_angle = 0.9;
  // Start sucking here
  pickup_params.suck_angle = 0.8;
  // Go back down to here to finish sucking.
  pickup_params.suck_angle_finish = 0.4;
  // Pack the box back in here.
  pickup_params.pickup_finish_angle = kClawTotePackAngle;
  pickup_params.intake_time = 0.70;
  pickup_params.intake_voltage = 7.0;

  if (ShouldExitAuto()) return;
  InitializeEncoders();
  ResetDrivetrain();

  WaitForClawZero();
  WaitForFridgeZero();

  // Initialize the fridge.
  MoveFridge(0.0, 0.3, true, kFridgeXProfile, kFridgeYProfile);

  LOG(INFO, "Lowering claw into position.\n");
  SetClawState(0.0, 2.0, false, kInstantaneousClaw);

  LOG(INFO, "Sucking in tote.\n");
  SetClawState(0.0, 6.0, true, kInstantaneousClaw);

  time::SleepFor(time::Time::InSeconds(0.7));
  LOG(INFO, "Done sucking in tote\n");

  // Now pick it up
  pickup = actors::MakePickupAction(pickup_params);
  pickup->Start();

  time::SleepFor(time::Time::InSeconds(0.9));
  // Start turning.
  LOG(INFO, "Turning in place\n");
  drive = SetDriveGoal(0.0, kFastDrive, -0.23, kStackingFirstTurn);

  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  LOG(INFO, "Now driving next to the can\n");
  drive = SetDriveGoal(0.60, kFastDrive);

  WaitUntilDoneOrCanceled(::std::move(pickup));
  if (ShouldExitAuto()) return;

  // Now grab it in the fridge.
  {
    actors::StackParams params;

    params.claw_out_angle = kClawTotePackAngle;
    params.bottom = 0.020;
    params.only_place = false;
    params.arm_clearance = kArmRaiseLowerClearance;
    params.over_box_before_place_height = 0.39;

    stack = actors::MakeStackAction(params);
    stack->Start();
  }
  WaitUntilDoneOrCanceled(::std::move(stack));
  if (ShouldExitAuto()) return;

  // Lower the claw to knock the tote.
  LOG(INFO, "Lowering the claw to knock the tote\n");
  SetClawStateNoWait(0.0, 0.0, true, kFastClaw);

  time::SleepFor(time::Time::InSeconds(0.1));
  if (ShouldExitAuto()) return;

  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  LOG(INFO, "Knocking the can over\n");
  drive = SetDriveGoal(0.40, kFastKnockDrive, 1.05, kComboTurn);
  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;
  {
    actors::HeldToLiftParams params;
    params.arm_clearance = kArmRaiseLowerClearance;
    params.clamp_pause_time = 0.1;
    params.before_lift_settle_time = 0.1;
    params.bottom_height = 0.020;
    params.claw_out_angle = kClawStackClearance;
    params.lift_params.lift_height = kStackUpHeight;
    params.lift_params.lift_arm = kStackUpArm;
    params.lift_params.second_lift = false;

    lift = actors::MakeHeldToLiftAction(params);
    lift->Start();
  }

  LOG(INFO, "Turning back to aim\n");
  drive = SetDriveGoal(0.0, kFastDrive, -0.70);
  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  SetClawStateNoWait(0.0, 4.0, false, kFastClaw);
  LOG(INFO, "Now driving the second tote\n");
  drive = SetDriveGoal(1.05, kFastDrive);

  // Wait until we are almost at the tote, and then start intaking.
  WaitUntilNear(0.35);

  SetClawState(0.0, 6.0, true, kFastClaw);
  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  if (ShouldExitAuto()) return;
  time::SleepFor(time::Time::InSeconds(0.30));
  if (ShouldExitAuto()) return;

  SetClawStateNoWait(0.0, 4.0, true, kFastClaw);
  if (ShouldExitAuto()) return;
  time::SleepFor(time::Time::InSeconds(0.10));

  WaitUntilDoneOrCanceled(::std::move(lift));
  if (ShouldExitAuto()) return;

  LOG(INFO, "Done sucking in tote\n");
  SetClawState(0.0, 0.0, true, kFastClaw);
  if (ShouldExitAuto()) return;

  // Now pick it up
  pickup = actors::MakePickupAction(pickup_params);
  pickup->Start();

  time::SleepFor(time::Time::InSeconds(1.0));
  if (ShouldExitAuto()) return;

  // Start turning.
  LOG(INFO, "Turning in place\n");
  drive = SetDriveGoal(0.0, kStackingSecondDrive, -0.40, kStackingSecondTurn);

  WaitUntilDoneOrCanceled(::std::move(pickup));
  if (ShouldExitAuto()) return;

  // Now grab it in the fridge.
  {
    actors::StackParams params;

    params.claw_out_angle = kClawTotePackAngle;
    params.bottom = 0.020;
    params.only_place = false;
    params.arm_clearance = kArmRaiseLowerClearance;
    params.over_box_before_place_height = 0.39;

    stack = actors::MakeStackAction(params);
    stack->Start();
  }

  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;
  LOG(INFO, "Driving next to the can.\n");
  drive = SetDriveGoal(0.65, kStackingSecondDrive);

  WaitUntilDoneOrCanceled(::std::move(stack));
  if (ShouldExitAuto()) return;

  // Lower the claw to knock the tote.
  LOG(INFO, "Lowering the claw to knock the tote\n");
  SetClawStateNoWait(0.0, 0.0, true, kFastClaw);

  // Lift the fridge.
  MoveFridge(0.0, 0.3, true, kFridgeXProfile, kFridgeYProfile);

  time::SleepFor(time::Time::InSeconds(0.1));
  if (ShouldExitAuto()) return;

  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  LOG(INFO, "Knocking the can over\n");
  drive = SetDriveGoal(0.40, kFastKnockDrive, 1.05, kComboTurn);
  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  LOG(INFO, "Turning back to aim\n");
  drive = SetDriveGoal(0.0, kFastDrive, -0.60);
  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;


  SetClawStateNoWait(0.0, 4.0, false, kFastClaw);
  LOG(INFO, "Now driving to the last tote\n");
  drive = SetDriveGoal(1.05, kFastDrive);
  WaitUntilNear(0.05);

  SetClawState(0.0, 7.0, true, kFastClaw);
  if (ShouldExitAuto()) return;

  time::SleepFor(time::Time::InSeconds(0.2));
  if (ShouldExitAuto()) return;

  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;
  SetClawState(0.0, 6.0, true, kFastClaw);

  LOG(INFO, "Racing over\n");
  //StepDrive(2.5, -1.4);
  drive = SetDriveGoal(2.5, kRaceDrive, -1.4, kRaceTurn);

  time::SleepFor(time::Time::InSeconds(0.5));

  LOG(INFO, "Moving totes out\n");
  MoveFridge(0.6, 0.32, true, kFridgeXProfile, kFridgeYProfile);

  WaitForFridge();
  if (ShouldExitAuto()) return;

  LOG(INFO, "Lowering totes\n");
  MoveFridge(0.6, 0.15, false, kFridgeXProfile, kFridgeYProfile);

  WaitForFridge();
  if (ShouldExitAuto()) return;

  time::SleepFor(time::Time::InSeconds(0.1));

  if (ShouldExitAuto()) return;

  LOG(INFO, "Retracting\n");
  MoveFridge(0.0, 0.10, false, kFridgeFastXProfile, kFridgeYProfile);

  SetClawState(0.0, 0.0, false, kFastClaw);

  if (ShouldExitAuto()) return;

  WaitUntilDoneOrCanceled(::std::move(drive));
  if (ShouldExitAuto()) return;

  LOG(INFO, "Backing away to let the stack ago\n");
  drive = SetDriveGoal(-0.1, kRaceBackupDrive);
  WaitUntilDoneOrCanceled(::std::move(drive));

  WaitForFridge();
  if (ShouldExitAuto()) return;
}

void GrabberForTime(double voltage, double wait_time) {
  ::aos::time::Time now = ::aos::time::Time::Now();
  ::aos::time::Time end_time = now + time::Time::InSeconds(wait_time);
  LOG(INFO, "Starting to grab at %f for %f seconds\n", voltage, wait_time);
  while (true) {
    autonomous::can_control.MakeWithBuilder().can_voltage(voltage).Send();
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

void CanGrabberAuto() {
  ResetDrivetrain();
  GrabberForTime(12.0, 0.18);
  if (ShouldExitAuto()) return;

  //GrabberForTime(4.0, 0.10);
  if (ShouldExitAuto()) return;
  InitializeEncoders();
  ResetDrivetrain();
  if (ShouldExitAuto()) return;
  drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(true)
      //.highgear(false)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(left_initial_position + 1.5)
      .left_velocity_goal(0)
      .right_goal(right_initial_position + 1.5)
      .right_velocity_goal(0)
      .Send();
  GrabberForTime(12.0, 0.02);

  GrabberForTime(4.0, 14.0);
  if (ShouldExitAuto()) return;
}

void HandleAuto() {
  ::aos::time::Time start_time = ::aos::time::Time::Now();
  LOG(INFO, "Starting auto mode at %f\n", start_time.ToSeconds());
  //TripleCanAuto();
  CanGrabberAuto();
}

}  // namespace autonomous
}  // namespace y2015
