#include <stdio.h>

#include <memory>

#include "aos/common/util/phased_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"

#include "frc971/autonomous/auto.q.h"
#include "frc971/constants.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/actors/drivetrain_actor.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/fridge/fridge.q.h"
#include "frc971/actors/pickup_actor.h"
#include "frc971/actors/stack_actor.h"

using ::aos::time::Time;
using ::frc971::control_loops::claw_queue;
using ::frc971::control_loops::fridge_queue;

namespace frc971 {
namespace autonomous {

constexpr double kClawAutoVelocity = 3.00;
constexpr double kClawAutoAcceleration = 6.0;
constexpr double kAngleEpsilon = 0.10;
double kClawTotePackAngle = 0.95;

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
  control_loops::drivetrain_queue.goal.MakeWithBuilder()
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
  control_loops::drivetrain_queue.goal.MakeWithBuilder()
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

void DriveSpin(double radians) {
  LOG(INFO, "going to spin %f\n", radians);

  // TODO(sensors): update this time thing maybe?
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
    ::aos::time::PhasedLoopXMS(5, 2500);
    driveTrainState = profile.Update(side_offset, goal_velocity);

    if (::std::abs(driveTrainState(0, 0) - side_offset) < epsilon) break;
    if (ShouldExitAuto()) return;

    LOG(DEBUG, "Driving left to %f, right to %f\n",
        left_initial_position - driveTrainState(0, 0),
        right_initial_position + driveTrainState(0, 0));
    control_loops::drivetrain_queue.goal.MakeWithBuilder()
        .control_loop_driving(true)
        //.highgear(false)
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

void WaitUntilDoneOrCanceled(aos::common::actions::Action *action) {
  while (true) {
    // Poll the running bit and auto done bits.
    ::aos::time::PhasedLoopXMS(5, 2500);
    if (!action->Running() || ShouldExitAuto()) {
      return;
    }
  }
}

::std::unique_ptr<::frc971::actors::DrivetrainAction> SetDriveGoal(
    double distance, bool slow_acceleration, double maximum_velocity = 0.7,
    double theta = 0) {
  LOG(INFO, "Driving to %f\n", distance);

  ::frc971::actors::DrivetrainActionParams params;
  params.left_initial_position = left_initial_position;
  params.right_initial_position = right_initial_position;
  params.y_offset = distance;
  params.theta_offset = theta;
  params.maximum_velocity = maximum_velocity;
  params.maximum_acceleration = slow_acceleration ? 0.3 : 0.2;
  auto drivetrain_action = actors::MakeDrivetrainAction(params);

  drivetrain_action->Start();
  left_initial_position +=
      distance - theta * constants::GetValues().turn_width / 2.0;
  right_initial_position +=
      distance + theta * constants::GetValues().turn_width / 2.0;
  return ::std::move(drivetrain_action);
}

void InitializeEncoders() {
  control_loops::drivetrain_queue.status.FetchLatest();
  while (!control_loops::drivetrain_queue.status.get()) {
    LOG(WARNING,
        "No previous drivetrain position packet, trying to fetch again\n");
    control_loops::drivetrain_queue.status.FetchNextBlocking();
  }
  left_initial_position =
      control_loops::drivetrain_queue.status->filtered_left_position;
  right_initial_position =
      control_loops::drivetrain_queue.status->filtered_right_position;
}

void SetClawState(double angle, double intake_voltage, double rollers_closed) {
  auto message = control_loops::claw_queue.goal.MakeMessage();
  message->angle = angle;
  message->max_velocity = kClawAutoVelocity;
  message->max_acceleration = kClawAutoAcceleration;
  message->angular_velocity = 0.0;
  message->intake = intake_voltage;
  message->rollers_closed = rollers_closed;

  LOG_STRUCT(DEBUG, "Sending claw goal", *message);
  message.Send();

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
  }
}

void HandleAuto() {
  ::aos::time::Time start_time = ::aos::time::Time::Now();
  LOG(INFO, "Handling auto mode at %f\n", start_time.ToSeconds());
  ::std::unique_ptr<::frc971::actors::DrivetrainAction> drive;
  ::std::unique_ptr<::frc971::actors::PickupAction> pickup;
  ::std::unique_ptr<::frc971::actors::StackAction> stack;
  // TODO(austin): Score!
  //::std::unique_ptr<ScoreAction> score;

  ResetDrivetrain();

  if (ShouldExitAuto()) return;
  InitializeEncoders();

  {
    auto new_fridge_goal = fridge_queue.goal.MakeMessage();
    new_fridge_goal->max_velocity = 0.0;
    new_fridge_goal->max_acceleration = 0.0;
    new_fridge_goal->profiling_type = 0;
    new_fridge_goal->height = 0.3;
    new_fridge_goal->velocity = 0.0;
    new_fridge_goal->max_angular_velocity = 0.0;
    new_fridge_goal->max_angular_acceleration = 0.0;
    new_fridge_goal->angle = 0.0;
    new_fridge_goal->angular_velocity = 0.0;
    new_fridge_goal->grabbers.top_front = true;
    new_fridge_goal->grabbers.top_back = true;
    new_fridge_goal->grabbers.bottom_front = true;
    new_fridge_goal->grabbers.bottom_back = true;

    if (!new_fridge_goal.Send()) {
      LOG(ERROR, "Sending fridge goal failed.\n");
      return;
    }
  }

  // Pick up the tote.
  SetClawState(0.0, 7.0, true);
  if (ShouldExitAuto()) return;
  time::SleepFor(time::Time::InSeconds(0.1));
  if (ShouldExitAuto()) return;

  // now pick it up
  {
    actors::PickupParams params;
    // Lift to here initially.
    params.pickup_angle = 0.9;
    // Start sucking here
    params.suck_angle = 0.8;
    // Go back down to here to finish sucking.
    params.suck_angle_finish = 0.4;
    // Pack the box back in here.
    params.pickup_finish_angle = kClawTotePackAngle;
    params.intake_time = 0.8;
    params.intake_voltage = 7.0;
    pickup = actors::MakePickupAction(params);
    pickup->Start();
    WaitUntilDoneOrCanceled(pickup.get());
  }
  if (ShouldExitAuto()) return;

  drive = SetDriveGoal(1.0, false);
  WaitUntilDoneOrCanceled(drive.get());

  SetClawState(0.0, 0.0, true);
  return;

  if (ShouldExitAuto()) return;

  if (false) {
    // drive up to the next tote
    drive = SetDriveGoal(1.0, false);
    WaitUntilDoneOrCanceled(drive.get());
    if (ShouldExitAuto()) return;

    // suck in the tote
    SetClawState(0.0, 7.0, true);
    drive = SetDriveGoal(0.2, false);
    WaitUntilDoneOrCanceled(drive.get());
    SetClawState(0.0, 0.0, true);
    if (ShouldExitAuto()) return;


    // we should have the tote, now stack it
    {
      actors::StackParams params;
      params.claw_out_angle = 0.6;
      params.bottom = 0.020;
      params.over_box_before_place_height = 0.39;
      stack = actors::MakeStackAction(params);
      WaitUntilDoneOrCanceled(stack.get());
    }
    if (ShouldExitAuto()) return;

    // turn 90
    DriveSpin(M_PI / 4.0);
    if (ShouldExitAuto()) return;

    // place the new stack
    // TODO(austin): Score!
    // score = MakeScoreAction(score_params);
    // WaitUntilDoneOrCanceled(score.get());
  }
}

}  // namespace autonomous
}  // namespace frc971
