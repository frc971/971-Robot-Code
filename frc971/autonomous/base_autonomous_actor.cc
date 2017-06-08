#include "frc971/autonomous/base_autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"

using ::frc971::control_loops::drivetrain_queue;
using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;

namespace frc971 {
namespace autonomous {

BaseAutonomousActor::BaseAutonomousActor(
    AutonomousActionQueueGroup *s,
    const control_loops::drivetrain::DrivetrainConfig dt_config)
    : aos::common::actions::ActorBase<AutonomousActionQueueGroup>(s),
      dt_config_(dt_config),
      initial_drivetrain_({0.0, 0.0}) {}

void BaseAutonomousActor::ResetDrivetrain() {
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

void BaseAutonomousActor::InitializeEncoders() {
  drivetrain_queue.status.FetchAnother();
  initial_drivetrain_.left = drivetrain_queue.status->estimated_left_position;
  initial_drivetrain_.right = drivetrain_queue.status->estimated_right_position;
}

void BaseAutonomousActor::StartDrive(double distance, double angle,
                                     ProfileParameters linear,
                                     ProfileParameters angular) {
  LOG(INFO, "Driving distance %f, angle %f\n", distance, angle);
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

void BaseAutonomousActor::WaitUntilDoneOrCanceled(
    ::std::unique_ptr<aos::common::actions::Action> action) {
  if (!action) {
    LOG(ERROR, "No action, not waiting\n");
    return;
  }

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    // Poll the running bit and see if we should cancel.
    phased_loop.SleepUntilNext();
    if (!action->Running() || ShouldCancel()) {
      return;
    }
  }
}

bool BaseAutonomousActor::WaitForDriveDone() {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);

  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_queue.status.FetchLatest();
    if (IsDriveDone()) {
      return true;
    }
  }
}

bool BaseAutonomousActor::IsDriveDone() {
  static constexpr double kPositionTolerance = 0.02;
  static constexpr double kVelocityTolerance = 0.10;
  static constexpr double kProfileTolerance = 0.001;

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
  return false;
}

bool BaseAutonomousActor::WaitForAboveAngle(double angle) {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_queue.status.FetchLatest();
    if (IsDriveDone()) {
      return true;
    }
    if (drivetrain_queue.status.get()) {
      if (drivetrain_queue.status->ground_angle > angle) {
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForBelowAngle(double angle) {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_queue.status.FetchLatest();
    if (IsDriveDone()) {
      return true;
    }
    if (drivetrain_queue.status.get()) {
      if (drivetrain_queue.status->ground_angle < angle) {
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForMaxBy(double angle) {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  double max_angle = -M_PI;
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_queue.status.FetchLatest();
    if (IsDriveDone()) {
      return true;
    }
    if (drivetrain_queue.status.get()) {
      if (drivetrain_queue.status->ground_angle > max_angle) {
        max_angle = drivetrain_queue.status->ground_angle;
      }
      if (drivetrain_queue.status->ground_angle < max_angle - angle) {
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForDriveNear(double distance, double angle) {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  constexpr double kPositionTolerance = 0.02;
  constexpr double kProfileTolerance = 0.001;

  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_queue.status.FetchLatest();
    if (drivetrain_queue.status.get()) {
      const double left_profile_error =
          (initial_drivetrain_.left -
           drivetrain_queue.status->profiled_left_position_goal);
      const double right_profile_error =
          (initial_drivetrain_.right -
           drivetrain_queue.status->profiled_right_position_goal);

      const double left_error =
          (initial_drivetrain_.left -
           drivetrain_queue.status->estimated_left_position);
      const double right_error =
          (initial_drivetrain_.right -
           drivetrain_queue.status->estimated_right_position);

      const double profile_distance_to_go =
          (left_profile_error + right_profile_error) / 2.0;
      const double profile_angle_to_go =
          (right_profile_error - left_profile_error) /
          (dt_config_.robot_radius * 2.0);

      const double distance_to_go = (left_error + right_error) / 2.0;
      const double angle_to_go =
          (right_error - left_error) / (dt_config_.robot_radius * 2.0);

      if (::std::abs(profile_distance_to_go) < distance + kProfileTolerance &&
          ::std::abs(profile_angle_to_go) < angle + kProfileTolerance &&
          ::std::abs(distance_to_go) < distance + kPositionTolerance &&
          ::std::abs(angle_to_go) < angle + kPositionTolerance) {
        LOG(INFO, "Closer than %f distance, %f angle\n", distance, angle);
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForDriveProfileDone() {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
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
                     initial_drivetrain_.right) < kProfileTolerance) {
        LOG(INFO, "Finished drive\n");
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForTurnProfileDone() {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  constexpr double kProfileTolerance = 0.001;

  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_queue.status.FetchLatest();
    if (drivetrain_queue.status.get()) {
      const double left_profile_error =
          (initial_drivetrain_.left -
           drivetrain_queue.status->profiled_left_position_goal);
      const double right_profile_error =
          (initial_drivetrain_.right -
           drivetrain_queue.status->profiled_right_position_goal);

      const double profile_angle_to_go =
          (right_profile_error - left_profile_error) /
          (dt_config_.robot_radius * 2.0);

      if (::std::abs(profile_angle_to_go) < kProfileTolerance) {
        LOG(INFO, "Finished turn profile\n");
        return true;
      }
    }
  }
}

double BaseAutonomousActor::DriveDistanceLeft() {
  using ::frc971::control_loops::drivetrain_queue;
  drivetrain_queue.status.FetchLatest();
  if (drivetrain_queue.status.get()) {
    const double left_error =
        (initial_drivetrain_.left -
         drivetrain_queue.status->estimated_left_position);
    const double right_error =
        (initial_drivetrain_.right -
         drivetrain_queue.status->estimated_right_position);

    return (left_error + right_error) / 2.0;
  } else {
    return 0;
  }
}

::std::unique_ptr<AutonomousAction> MakeAutonomousAction(
    const AutonomousActionParams &params) {
  return ::std::unique_ptr<AutonomousAction>(
      new AutonomousAction(&autonomous_action, params));
}

}  // namespace autonomous
}  // namespace frc971
