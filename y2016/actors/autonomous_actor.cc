#include "y2016/actors/autonomous_actor.h"

#include <inttypes.h>

#include <cmath>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2016/control_loops/drivetrain/drivetrain_base.h"
#include "y2016/control_loops/superstructure/superstructure.q.h"
#include "y2016/actors/autonomous_action.q.h"

namespace y2016 {
namespace actors {
using ::frc971::control_loops::drivetrain_queue;

namespace {
const ProfileParameters kSlowDrive = {1.0, 1.0};
const ProfileParameters kFastDrive = {3.0, 2.5};

const ProfileParameters kSlowTurn = {1.7, 3.0};
const ProfileParameters kFastTurn = {3.0, 10.0};
}  // namespace

AutonomousActor::AutonomousActor(actors::AutonomousActionQueueGroup *s)
    : aos::common::actions::ActorBase<actors::AutonomousActionQueueGroup>(s),
      dt_config_(control_loops::drivetrain::GetDrivetrainConfig()),
      initial_drivetrain_({0.0, 0.0}) {}

void AutonomousActor::ResetDrivetrain() {
  LOG(INFO, "resetting the drivetrain\n");
  drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .highgear(true)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(initial_drivetrain_.left)
      .left_velocity_goal(0)
      .right_goal(initial_drivetrain_.right)
      .right_velocity_goal(0)
      .Send();
}

void AutonomousActor::StartDrive(double distance, double angle,
                                 ProfileParameters linear,
                                 ProfileParameters angular) {
  {
    {
      const double dangle = angle * dt_config_.robot_radius;
      initial_drivetrain_.left += distance - dangle;
      initial_drivetrain_.right += distance + dangle;
    }

    auto drivetrain_message = drivetrain_queue.goal.MakeMessage();
    drivetrain_message->control_loop_driving = true;
    drivetrain_message->highgear = true;
    drivetrain_message->steering = 0.0;
    drivetrain_message->throttle = 0.0;
    drivetrain_message->left_goal = initial_drivetrain_.left;
    drivetrain_message->left_velocity_goal = 0;
    drivetrain_message->right_goal = initial_drivetrain_.right;
    drivetrain_message->right_velocity_goal = 0;
    drivetrain_message->linear = linear;
    drivetrain_message->angular = angular;

    LOG_STRUCT(DEBUG, "drivetrain_goal", *drivetrain_message);

    drivetrain_message.Send();
  }
}

void AutonomousActor::InitializeEncoders() {
  drivetrain_queue.status.FetchAnother();
  initial_drivetrain_.left = drivetrain_queue.status->estimated_left_position;
  initial_drivetrain_.right = drivetrain_queue.status->estimated_right_position;
}

void AutonomousActor::WaitUntilDoneOrCanceled(
    ::std::unique_ptr<aos::common::actions::Action> action) {
  if (!action) {
    LOG(ERROR, "No action, not waiting\n");
    return;
  }

  ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(5),
                                      ::aos::time::Time::InMS(5) / 2);
  while (true) {
    // Poll the running bit and see if we should cancel.
    phased_loop.SleepUntilNext();
    if (!action->Running() || ShouldCancel()) {
      return;
    }
  }
}

bool AutonomousActor::WaitForDriveDone() {
  ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(5),
                                      ::aos::time::Time::InMS(5) / 2);
  constexpr double kPositionTolerance = 0.02;
  constexpr double kVelocityTolerance = 0.10;
  constexpr double kProfileTolerance = 0.001;

  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_queue.status.FetchLatest();
    if (drivetrain_queue.status.get()) {
      if (::std::abs(drivetrain_queue.status->profiled_left_position_goal -
                     initial_drivetrain_.left) < kProfileTolerance &&
          ::std::abs(drivetrain_queue.status->profiled_right_position_goal -
                     initial_drivetrain_.right) < kProfileTolerance &&
          ::std::abs(drivetrain_queue.status->estimated_left_position -
                     initial_drivetrain_.left) < kPositionTolerance &&
          ::std::abs(drivetrain_queue.status->estimated_right_position -
                     initial_drivetrain_.right) < kPositionTolerance &&
          ::std::abs(drivetrain_queue.status->estimated_left_velocity) <
              kVelocityTolerance &&
          ::std::abs(drivetrain_queue.status->estimated_right_velocity) <
              kVelocityTolerance) {
        LOG(INFO, "Finished drive\n");
        return true;
      }
    }
  }
}

void AutonomousActor::MoveSuperstructure(
    double intake, double shoulder, double wrist,
    const ProfileParameters intake_params,
    const ProfileParameters shoulder_params,
    const ProfileParameters wrist_params, double top_rollers,
    double bottom_rollers) {
  superstructure_goal_ = {intake, shoulder, wrist};

  auto new_superstructure_goal =
      ::y2016::control_loops::superstructure_queue.goal.MakeMessage();

  new_superstructure_goal->angle_intake = intake;
  new_superstructure_goal->angle_shoulder = shoulder;
  new_superstructure_goal->angle_wrist = wrist;

  new_superstructure_goal->max_angular_velocity_intake =
      intake_params.max_velocity;
  new_superstructure_goal->max_angular_velocity_shoulder =
      shoulder_params.max_velocity;
  new_superstructure_goal->max_angular_velocity_wrist =
      wrist_params.max_velocity;

  new_superstructure_goal->max_angular_acceleration_intake =
      intake_params.max_acceleration;
  new_superstructure_goal->max_angular_acceleration_shoulder =
      shoulder_params.max_acceleration;
  new_superstructure_goal->max_angular_acceleration_wrist =
      wrist_params.max_acceleration;

  new_superstructure_goal->voltage_top_rollers = top_rollers;
  new_superstructure_goal->voltage_bottom_rollers = bottom_rollers;

  if (!new_superstructure_goal.Send()) {
    LOG(ERROR, "Sending superstructure goal failed.\n");
  }
}

void AutonomousActor::WaitForSuperstructure() {
  while (true) {
    if (ShouldCancel()) return;
    control_loops::superstructure_queue.status.FetchAnother();

    constexpr double kProfileError = 1e-5;
    constexpr double kEpsilon = 0.03;

    if (control_loops::superstructure_queue.status->state < 12 ||
        control_loops::superstructure_queue.status->state == 16) {
      LOG(ERROR, "Superstructure no longer running, aborting action\n");
      return;
    }

    if (::std::abs(
            control_loops::superstructure_queue.status->intake.goal_angle -
            superstructure_goal_.intake) < kProfileError &&
        ::std::abs(
            control_loops::superstructure_queue.status->shoulder.goal_angle -
            superstructure_goal_.shoulder) < kProfileError &&
        ::std::abs(
            control_loops::superstructure_queue.status->wrist.goal_angle -
            superstructure_goal_.wrist) < kProfileError &&
        ::std::abs(control_loops::superstructure_queue.status->intake
                       .goal_angular_velocity) < kProfileError &&
        ::std::abs(control_loops::superstructure_queue.status->shoulder
                       .goal_angular_velocity) < kProfileError &&
        ::std::abs(control_loops::superstructure_queue.status->wrist
                       .goal_angular_velocity) < kProfileError) {
      LOG(INFO, "Profile done.\n");
      if (::std::abs(control_loops::superstructure_queue.status->intake.angle -
                     superstructure_goal_.intake) < kEpsilon &&
          ::std::abs(
              control_loops::superstructure_queue.status->shoulder.angle -
              superstructure_goal_.shoulder) < kEpsilon &&
          ::std::abs(control_loops::superstructure_queue.status->wrist.angle -
                     superstructure_goal_.wrist) < kEpsilon &&
          ::std::abs(control_loops::superstructure_queue.status->intake
                         .angular_velocity) < kEpsilon &&
          ::std::abs(control_loops::superstructure_queue.status->shoulder
                         .angular_velocity) < kEpsilon &&
          ::std::abs(control_loops::superstructure_queue.status->wrist
                         .angular_velocity) < kEpsilon) {
        LOG(INFO, "Near goal, done.\n");
        return;
      }
    }
  }
}

bool AutonomousActor::RunAction(const actors::AutonomousActionParams &params) {
  LOG(INFO, "Starting autonomous action with mode %" PRId32 "\n", params.mode);

  InitializeEncoders();
  ResetDrivetrain();

  StartDrive(1.0, 0.0, kSlowDrive, kSlowTurn);

  if (!WaitForDriveDone()) return true;

  StartDrive(0.0, M_PI, kSlowDrive, kSlowTurn);

  if (!WaitForDriveDone()) return true;

  ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(5),
                                      ::aos::time::Time::InMS(5) / 2);
  while (!ShouldCancel()) {
    phased_loop.SleepUntilNext();
  }
  LOG(DEBUG, "Done running\n");

  return true;
}

::std::unique_ptr<AutonomousAction> MakeAutonomousAction(
    const ::y2016::actors::AutonomousActionParams &params) {
  return ::std::unique_ptr<AutonomousAction>(
      new AutonomousAction(&::y2016::actors::autonomous_action, params));
}

}  // namespace actors
}  // namespace y2016
