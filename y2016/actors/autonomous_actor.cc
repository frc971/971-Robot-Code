#include "y2016/actors/autonomous_actor.h"

#include <inttypes.h>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2016/control_loops/drivetrain/drivetrain_base.h"

namespace y2016 {
namespace actors {
using ::frc971::control_loops::ProfileParameters;
using ::frc971::control_loops::drivetrain_queue;

namespace {
const ProfileParameters kSlowDrive = {1.0, 1.0};
const ProfileParameters kFastDrive = {3.0, 2.5};

const ProfileParameters kSlowTurn = {1.7, 3.0};
const ProfileParameters kFastTurn = {3.0, 10.0};
}  // namespace

AutonomousActor::AutonomousActor(actors::AutonomousActionQueueGroup *s)
    : aos::common::actions::ActorBase<actors::AutonomousActionQueueGroup>(s),
      left_initial_position_(0.0),
      right_initial_position_(0.0),
      dt_config_(control_loops::drivetrain::GetDrivetrainConfig()) {}

void AutonomousActor::ResetDrivetrain() {
  LOG(INFO, "resetting the drivetrain\n");
  drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .highgear(true)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(left_initial_position_)
      .left_velocity_goal(0)
      .right_goal(right_initial_position_)
      .right_velocity_goal(0)
      .Send();
}

void AutonomousActor::StartDrive(double distance, double angle,
                                 ProfileParameters linear,
                                 ProfileParameters angular) {
  {
    {
      const double dangle = angle * dt_config_.robot_radius;
      left_initial_position_ += distance - dangle;
      right_initial_position_ += distance + dangle;
    }

    auto drivetrain_message =
        drivetrain_queue.goal.MakeMessage();
    drivetrain_message->control_loop_driving = true;
    drivetrain_message->highgear = true;
    drivetrain_message->steering = 0.0;
    drivetrain_message->throttle = 0.0;
    drivetrain_message->left_goal = left_initial_position_;
    drivetrain_message->left_velocity_goal = 0;
    drivetrain_message->right_goal = right_initial_position_;
    drivetrain_message->right_velocity_goal = 0;
    drivetrain_message->linear = linear;
    drivetrain_message->angular = angular;

    LOG_STRUCT(DEBUG, "drivetrain_goal", *drivetrain_message);

    drivetrain_message.Send();
  }
}

void AutonomousActor::InitializeEncoders() {
  drivetrain_queue.status.FetchAnother();
  left_initial_position_ = drivetrain_queue.status->estimated_left_position;
  right_initial_position_ = drivetrain_queue.status->estimated_right_position;
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
                     left_initial_position_) < kProfileTolerance &&
          ::std::abs(drivetrain_queue.status->profiled_right_position_goal -
                     right_initial_position_) < kProfileTolerance &&
          ::std::abs(drivetrain_queue.status->estimated_left_position -
                     left_initial_position_) < kPositionTolerance &&
          ::std::abs(drivetrain_queue.status->estimated_right_position -
                     right_initial_position_) < kPositionTolerance &&
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
