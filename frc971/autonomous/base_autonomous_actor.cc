#include "frc971/autonomous/base_autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/util/phased_loop.h"
#include "aos/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"

using ::frc971::control_loops::drivetrain_queue;
using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;

namespace frc971 {
namespace autonomous {

BaseAutonomousActor::BaseAutonomousActor(
    AutonomousActionQueueGroup *s,
    const control_loops::drivetrain::DrivetrainConfig<double> &dt_config)
    : aos::common::actions::ActorBase<AutonomousActionQueueGroup>(s),
      dt_config_(dt_config),
      initial_drivetrain_({0.0, 0.0}) {}

void BaseAutonomousActor::ResetDrivetrain() {
  LOG(INFO, "resetting the drivetrain\n");
  max_drivetrain_voltage_ = 12.0;
  drivetrain_queue.goal.MakeWithBuilder()
      .controller_type(0)
      .highgear(true)
      .wheel(0.0)
      .throttle(0.0)
      .left_goal(initial_drivetrain_.left)
      .right_goal(initial_drivetrain_.right)
      .max_ss_voltage(max_drivetrain_voltage_)
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
  drivetrain_message->controller_type = 1;
  drivetrain_message->highgear = true;
  drivetrain_message->wheel = 0.0;
  drivetrain_message->throttle = 0.0;
  drivetrain_message->left_goal = initial_drivetrain_.left;
  drivetrain_message->right_goal = initial_drivetrain_.right;
  drivetrain_message->max_ss_voltage = max_drivetrain_voltage_;
  drivetrain_message->linear = linear;
  drivetrain_message->angular = angular;

  LOG_STRUCT(DEBUG, "dtg", *drivetrain_message);

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

  bool drive_has_been_close = false;
  bool turn_has_been_close = false;
  bool printed_first = false;

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

      const bool drive_close =
          ::std::abs(profile_distance_to_go) < distance + kProfileTolerance &&
          ::std::abs(distance_to_go) < distance + kPositionTolerance;
      const bool turn_close =
          ::std::abs(profile_angle_to_go) < angle + kProfileTolerance &&
          ::std::abs(angle_to_go) < angle + kPositionTolerance;

      drive_has_been_close |= drive_close;
      turn_has_been_close |= turn_close;
      if (drive_has_been_close && !turn_has_been_close && !printed_first) {
        LOG(INFO, "Drive finished first\n");
        printed_first = true;
      } else if (!drive_has_been_close && turn_has_been_close &&
                 !printed_first) {
        LOG(INFO, "Turn finished first\n");
        printed_first = true;
      }

      if (drive_close && turn_close) {
        LOG(INFO, "Closer than %f < %f distance, %f < %f angle\n",
            distance_to_go, distance, angle_to_go, angle);
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForDriveProfileNear(double tolerance) {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_queue.status.FetchLatest();

    const Eigen::Matrix<double, 7, 1> current_error =
        (Eigen::Matrix<double, 7, 1>()
             << initial_drivetrain_.left -
                    drivetrain_queue.status->profiled_left_position_goal,
         0.0, initial_drivetrain_.right -
                  drivetrain_queue.status->profiled_right_position_goal,
         0.0, 0.0, 0.0, 0.0)
            .finished();
    const Eigen::Matrix<double, 2, 1> linear_error =
        dt_config_.LeftRightToLinear(current_error);

    if (drivetrain_queue.status.get()) {
      if (::std::abs(linear_error(0)) < tolerance) {
        LOG(INFO, "Finished drive\n");
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForDriveProfileDone() {
  constexpr double kProfileTolerance = 0.001;
  return WaitForDriveProfileNear(kProfileTolerance);
}

bool BaseAutonomousActor::WaitForTurnProfileNear(double tolerance) {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_queue.status.FetchLatest();

    const Eigen::Matrix<double, 7, 1> current_error =
        (Eigen::Matrix<double, 7, 1>()
             << initial_drivetrain_.left -
                    drivetrain_queue.status->profiled_left_position_goal,
         0.0, initial_drivetrain_.right -
                  drivetrain_queue.status->profiled_right_position_goal,
         0.0, 0.0, 0.0, 0.0)
            .finished();
    const Eigen::Matrix<double, 2, 1> angular_error =
        dt_config_.LeftRightToAngular(current_error);

    if (drivetrain_queue.status.get()) {
      if (::std::abs(angular_error(0)) < tolerance) {
        LOG(INFO, "Finished turn\n");
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForTurnProfileDone() {
  constexpr double kProfileTolerance = 0.001;
  return WaitForTurnProfileNear(kProfileTolerance);
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

BaseAutonomousActor::SplineHandle BaseAutonomousActor::PlanSpline(
    const ::frc971::MultiSpline &spline) {
  LOG(INFO, "Planning spline\n");

  int32_t spline_handle = (++spline_handle_) | ((getpid() & 0xFFFF) << 15);

  drivetrain_queue.goal.FetchLatest();

  auto drivetrain_message = drivetrain_queue.goal.MakeMessage();
  drivetrain_message->controller_type = 2;

  drivetrain_message->spline = spline;
  drivetrain_message->spline.spline_idx = spline_handle;
  drivetrain_message->spline_handle = goal_spline_handle_;

  LOG_STRUCT(DEBUG, "dtg", *drivetrain_message);

  drivetrain_message.Send();

  return BaseAutonomousActor::SplineHandle(spline_handle, this);
}

bool BaseAutonomousActor::SplineHandle::IsPlanned() {
  drivetrain_queue.status.FetchLatest();
  LOG_STRUCT(INFO, "dts", *drivetrain_queue.status.get());
  if (drivetrain_queue.status.get() &&
      ((drivetrain_queue.status->trajectory_logging.planning_spline_idx ==
            spline_handle_ &&
        drivetrain_queue.status->trajectory_logging.planning_state == 3) ||
       drivetrain_queue.status->trajectory_logging.current_spline_idx ==
           spline_handle_)) {
    return true;
  }
  return false;
}

bool BaseAutonomousActor::SplineHandle::WaitForPlan() {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (base_autonomous_actor_->ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    if (IsPlanned()) {
      return true;
    }
  }
}

void BaseAutonomousActor::SplineHandle::Start() {
  auto drivetrain_message = drivetrain_queue.goal.MakeMessage();
  drivetrain_message->controller_type = 2;

  LOG(INFO, "Starting spline\n");

  drivetrain_message->spline_handle = spline_handle_;
  base_autonomous_actor_->goal_spline_handle_ = spline_handle_;

  LOG_STRUCT(DEBUG, "dtg", *drivetrain_message);

  drivetrain_message.Send();
}

bool BaseAutonomousActor::SplineHandle::IsDone() {
  drivetrain_queue.status.FetchLatest();
  LOG_STRUCT(INFO, "dts", *drivetrain_queue.status.get());

  if (drivetrain_queue.status.get() &&
      drivetrain_queue.status->trajectory_logging.current_spline_idx ==
          spline_handle_) {
    return false;
  }
  return true;
}

bool BaseAutonomousActor::SplineHandle::WaitForDone() {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (base_autonomous_actor_->ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    if (IsDone()) {
      return true;
    }
  }
}

::std::unique_ptr<AutonomousAction> MakeAutonomousAction(
    const AutonomousActionParams &params) {
  return ::std::unique_ptr<AutonomousAction>(
      new AutonomousAction(&autonomous_action, params));
}

}  // namespace autonomous
}  // namespace frc971
