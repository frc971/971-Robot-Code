#include "y2016_bot3/actors/autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2016_bot3/control_loops/drivetrain/drivetrain_base.h"
#include "y2016_bot3/control_loops/intake/intake.q.h"
#include "y2016_bot3/actors/autonomous_action.q.h"
#include "y2016_bot3/queues/ball_detector.q.h"

namespace y2016_bot3 {
namespace actors {
using ::frc971::control_loops::drivetrain_queue;

namespace chrono = ::std::chrono;

namespace {
const ProfileParameters kLowBarDrive = {1.3, 2.5};
}  // namespace

AutonomousActor::AutonomousActor(actors::AutonomousActionQueueGroup *s)
    : aos::common::actions::ActorBase<actors::AutonomousActionQueueGroup>(s),
      dt_config_(control_loops::drivetrain::GetDrivetrainConfig()),
      initial_drivetrain_({0.0, 0.0}) {}

void AutonomousActor::ResetDrivetrain() {
  LOG(INFO, "resetting the drivetrain\n");
  drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(initial_drivetrain_.left)
      .left_velocity_goal(0)
      .right_goal(initial_drivetrain_.right)
      .right_velocity_goal(0)
      .Send();
}

void AutonomousActor::StartDrive(double distance, double angle,
                                 ProfileParameters linear) {
  {
    LOG(INFO, "Driving distance %f, angle %f\n", distance, angle);
    {
      const double dangle = angle * dt_config_.robot_radius;
      initial_drivetrain_.left += distance - dangle;
      initial_drivetrain_.right += distance + dangle;
    }

    auto drivetrain_message = drivetrain_queue.goal.MakeMessage();
    drivetrain_message->control_loop_driving = true;
    drivetrain_message->steering = 0.0;
    drivetrain_message->throttle = 0.0;
    drivetrain_message->left_goal = initial_drivetrain_.left;
    drivetrain_message->left_velocity_goal = 0;
    drivetrain_message->right_goal = initial_drivetrain_.right;
    drivetrain_message->right_velocity_goal = 0;
    drivetrain_message->linear = linear;

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

bool AutonomousActor::WaitForDriveNear(double distance, double angle) {
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

bool AutonomousActor::WaitForDriveProfileDone() {
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

bool AutonomousActor::IsDriveDone() {
  constexpr double kPositionTolerance = 0.02;
  constexpr double kVelocityTolerance = 0.10;
  constexpr double kProfileTolerance = 0.001;

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

bool AutonomousActor::WaitForDriveDone() {
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

void AutonomousActor::MoveIntake(double intake_goal,
                                 const ProfileParameters intake_params,
                                 bool traverse_up, double roller_power) {
  auto new_intake_goal =
      ::y2016_bot3::control_loops::intake_queue.goal.MakeMessage();

  new_intake_goal->angle_intake = intake_goal;

  new_intake_goal->max_angular_velocity_intake = intake_params.max_velocity;

  new_intake_goal->max_angular_acceleration_intake =
      intake_params.max_acceleration;

  new_intake_goal->voltage_top_rollers = roller_power;
  new_intake_goal->voltage_bottom_rollers = roller_power;

  new_intake_goal->traverse_down = !traverse_up;

  if (!new_intake_goal.Send()) {
    LOG(ERROR, "Sending intake goal failed.\n");
  }
}

bool AutonomousActor::IntakeDone() {
  control_loops::intake_queue.status.FetchAnother();

  constexpr double kProfileError = 1e-5;
  constexpr double kEpsilon = 0.15;

  if (control_loops::intake_queue.status->state < 12 ||
      control_loops::intake_queue.status->state == 16) {
    LOG(ERROR, "Intake no longer running, aborting action\n");
    return true;
  }

  if (::std::abs(control_loops::intake_queue.status->intake.goal_angle -
                 intake_goal_.intake) < kProfileError &&
      ::std::abs(
          control_loops::intake_queue.status->intake.goal_angular_velocity) <
          kProfileError) {
    LOG(DEBUG, "Profile done.\n");
    if (::std::abs(control_loops::intake_queue.status->intake.angle -
                   intake_goal_.intake) < kEpsilon &&
        ::std::abs(
            control_loops::intake_queue.status->intake.angular_velocity) <
            kEpsilon) {
      LOG(INFO, "Near goal, done.\n");
      return true;
    }
  }
  return false;
}

void AutonomousActor::WaitForIntake() {
  while (true) {
    if (ShouldCancel()) return;
    if (IntakeDone()) return;
  }
}

void AutonomousActor::LowBarDrive() {
  StartDrive(-5.5, 0.0, kLowBarDrive);

  if (!WaitForDriveNear(5.3, 0.0)) return;

  if (!WaitForDriveNear(5.0, 0.0)) return;

  StartDrive(0.0, 0.0, kLowBarDrive);

  if (!WaitForDriveNear(3.0, 0.0)) return;

  StartDrive(0.0, 0.0, kLowBarDrive);

  if (!WaitForDriveNear(1.0, 0.0)) return;

  StartDrive(0, -M_PI / 4.0 - 0.2, kLowBarDrive);
}

void AutonomousActor::WaitForBallOrDriveDone() {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) {
      return;
    }
    phased_loop.SleepUntilNext();
    drivetrain_queue.status.FetchLatest();
    if (IsDriveDone()) {
      return;
    }

    ::y2016_bot3::sensors::ball_detector.FetchLatest();
    if (::y2016_bot3::sensors::ball_detector.get()) {
      const bool ball_detected =
          ::y2016_bot3::sensors::ball_detector->voltage > 2.5;
      if (ball_detected) {
        return;
      }
    }
  }
}

void AutonomousActor::WaitForBall() {
  while (true) {
    ::y2016_bot3::sensors::ball_detector.FetchAnother();
    if (::y2016_bot3::sensors::ball_detector.get()) {
      const bool ball_detected =
          ::y2016_bot3::sensors::ball_detector->voltage > 2.5;
      if (ball_detected) {
        return;
      }
      if (ShouldCancel()) return;
    }
  }
}

bool AutonomousActor::RunAction(const actors::AutonomousActionParams &params) {
  aos::monotonic_clock::time_point start_time = aos::monotonic_clock::now();
  LOG(INFO, "Starting autonomous action with mode %" PRId32 "\n", params.mode);

  InitializeEncoders();
  ResetDrivetrain();

  switch (params.mode) {
    case 0:
    default:
      // TODO: Write auto code
      LOG(ERROR, "Uhh... someone implement this please :)");
      break;
  }

  if (!WaitForDriveDone()) return true;

  LOG(INFO, "Done %f\n", chrono::duration_cast<chrono::duration<double>>(
                             aos::monotonic_clock::now() - start_time).count());

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);

  while (!ShouldCancel()) {
    phased_loop.SleepUntilNext();
  }
  LOG(DEBUG, "Done running\n");

  return true;
}

::std::unique_ptr<AutonomousAction> MakeAutonomousAction(
    const ::y2016_bot3::actors::AutonomousActionParams &params) {
  return ::std::unique_ptr<AutonomousAction>(
      new AutonomousAction(&::y2016_bot3::actors::autonomous_action, params));
}

}  // namespace actors
}  // namespace y2016_bot3
