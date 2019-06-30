#include "frc971/autonomous/base_autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/util/phased_loop.h"
#include "aos/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/localizer.q.h"
#include "y2019/control_loops/drivetrain/target_selector.q.h"

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;

namespace frc971 {
namespace autonomous {

BaseAutonomousActor::BaseAutonomousActor(
    ::aos::EventLoop *event_loop,
    const control_loops::drivetrain::DrivetrainConfig<double> &dt_config)
    : aos::common::actions::ActorBase<AutonomousActionQueueGroup>(
          event_loop, ".frc971.autonomous.autonomous_action"),
      dt_config_(dt_config),
      initial_drivetrain_({0.0, 0.0}),
      target_selector_hint_sender_(
          event_loop->MakeSender<
              ::y2019::control_loops::drivetrain::TargetSelectorHint>(
              ".y2019.control_loops.drivetrain.target_selector_hint")),
      drivetrain_goal_sender_(
          event_loop
              ->MakeSender<::frc971::control_loops::DrivetrainQueue::Goal>(
                  ".frc971.control_loops.drivetrain_queue.goal")),
      drivetrain_status_fetcher_(
          event_loop
              ->MakeFetcher<::frc971::control_loops::DrivetrainQueue::Status>(
                  ".frc971.control_loops.drivetrain_queue.status")),
      drivetrain_goal_fetcher_(
          event_loop
              ->MakeFetcher<::frc971::control_loops::DrivetrainQueue::Goal>(
                  ".frc971.control_loops.drivetrain_queue.goal")) {}

void BaseAutonomousActor::ResetDrivetrain() {
  LOG(INFO, "resetting the drivetrain\n");
  max_drivetrain_voltage_ = 12.0;
  goal_spline_handle_ = 0;

  auto drivetrain_goal_message = drivetrain_goal_sender_.MakeMessage();
  drivetrain_goal_message->controller_type = 0;
  drivetrain_goal_message->highgear = true;
  drivetrain_goal_message->wheel = 0.0;
  drivetrain_goal_message->throttle = 0.0;
  drivetrain_goal_message->left_goal = initial_drivetrain_.left;
  drivetrain_goal_message->right_goal = initial_drivetrain_.right;
  drivetrain_goal_message->max_ss_voltage = max_drivetrain_voltage_;
  drivetrain_goal_message.Send();
}

void BaseAutonomousActor::InitializeEncoders() {
  // Spin until we get a message.
  WaitUntil([this]() { return drivetrain_status_fetcher_.Fetch(); });

  initial_drivetrain_.left =
      drivetrain_status_fetcher_->estimated_left_position;
  initial_drivetrain_.right =
      drivetrain_status_fetcher_->estimated_right_position;
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

  auto drivetrain_message = drivetrain_goal_sender_.MakeMessage();
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
                                      event_loop()->monotonic_now(),
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
                                      event_loop()->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);

  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_status_fetcher_.Fetch();
    if (IsDriveDone()) {
      return true;
    }
  }
}

bool BaseAutonomousActor::IsDriveDone() {
  static constexpr double kPositionTolerance = 0.02;
  static constexpr double kVelocityTolerance = 0.10;
  static constexpr double kProfileTolerance = 0.001;

  if (drivetrain_status_fetcher_.get()) {
    if (::std::abs(drivetrain_status_fetcher_->profiled_left_position_goal -
                   initial_drivetrain_.left) < kProfileTolerance &&
        ::std::abs(drivetrain_status_fetcher_->profiled_right_position_goal -
                   initial_drivetrain_.right) < kProfileTolerance &&
        ::std::abs(drivetrain_status_fetcher_->estimated_left_position -
                   initial_drivetrain_.left) < kPositionTolerance &&
        ::std::abs(drivetrain_status_fetcher_->estimated_right_position -
                   initial_drivetrain_.right) < kPositionTolerance &&
        ::std::abs(drivetrain_status_fetcher_->estimated_left_velocity) <
            kVelocityTolerance &&
        ::std::abs(drivetrain_status_fetcher_->estimated_right_velocity) <
            kVelocityTolerance) {
      LOG(INFO, "Finished drive\n");
      return true;
    }
  }
  return false;
}

bool BaseAutonomousActor::WaitForAboveAngle(double angle) {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      event_loop()->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_status_fetcher_.Fetch();
    if (IsDriveDone()) {
      return true;
    }
    if (drivetrain_status_fetcher_.get()) {
      if (drivetrain_status_fetcher_->ground_angle > angle) {
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForBelowAngle(double angle) {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      event_loop()->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_status_fetcher_.Fetch();
    if (IsDriveDone()) {
      return true;
    }
    if (drivetrain_status_fetcher_.get()) {
      if (drivetrain_status_fetcher_->ground_angle < angle) {
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForMaxBy(double angle) {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      event_loop()->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);
  double max_angle = -M_PI;
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_status_fetcher_.Fetch();
    if (IsDriveDone()) {
      return true;
    }
    if (drivetrain_status_fetcher_.get()) {
      if (drivetrain_status_fetcher_->ground_angle > max_angle) {
        max_angle = drivetrain_status_fetcher_->ground_angle;
      }
      if (drivetrain_status_fetcher_->ground_angle < max_angle - angle) {
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForDriveNear(double distance, double angle) {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      event_loop()->monotonic_now(),
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
    drivetrain_status_fetcher_.Fetch();
    if (drivetrain_status_fetcher_.get()) {
      const double left_profile_error =
          (initial_drivetrain_.left -
           drivetrain_status_fetcher_->profiled_left_position_goal);
      const double right_profile_error =
          (initial_drivetrain_.right -
           drivetrain_status_fetcher_->profiled_right_position_goal);

      const double left_error =
          (initial_drivetrain_.left -
           drivetrain_status_fetcher_->estimated_left_position);
      const double right_error =
          (initial_drivetrain_.right -
           drivetrain_status_fetcher_->estimated_right_position);

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
                                      event_loop()->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_status_fetcher_.Fetch();

    const Eigen::Matrix<double, 7, 1> current_error =
        (Eigen::Matrix<double, 7, 1>()
             << initial_drivetrain_.left -
                    drivetrain_status_fetcher_->profiled_left_position_goal,
         0.0, initial_drivetrain_.right -
                  drivetrain_status_fetcher_->profiled_right_position_goal,
         0.0, 0.0, 0.0, 0.0)
            .finished();
    const Eigen::Matrix<double, 2, 1> linear_error =
        dt_config_.LeftRightToLinear(current_error);

    if (drivetrain_status_fetcher_.get()) {
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
                                      event_loop()->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_status_fetcher_.Fetch();

    const Eigen::Matrix<double, 7, 1> current_error =
        (Eigen::Matrix<double, 7, 1>()
             << initial_drivetrain_.left -
                    drivetrain_status_fetcher_->profiled_left_position_goal,
         0.0, initial_drivetrain_.right -
                  drivetrain_status_fetcher_->profiled_right_position_goal,
         0.0, 0.0, 0.0, 0.0)
            .finished();
    const Eigen::Matrix<double, 2, 1> angular_error =
        dt_config_.LeftRightToAngular(current_error);

    if (drivetrain_status_fetcher_.get()) {
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
  drivetrain_status_fetcher_.Fetch();
  if (drivetrain_status_fetcher_.get()) {
    const double left_error =
        (initial_drivetrain_.left -
         drivetrain_status_fetcher_->estimated_left_position);
    const double right_error =
        (initial_drivetrain_.right -
         drivetrain_status_fetcher_->estimated_right_position);

    return (left_error + right_error) / 2.0;
  } else {
    return 0;
  }
}

bool BaseAutonomousActor::SplineHandle::SplineDistanceRemaining(
    double distance) {
  base_autonomous_actor_->drivetrain_status_fetcher_.Fetch();
  if (base_autonomous_actor_->drivetrain_status_fetcher_.get()) {
    return base_autonomous_actor_->drivetrain_status_fetcher_
               ->trajectory_logging.is_executing &&
           base_autonomous_actor_->drivetrain_status_fetcher_
                   ->trajectory_logging.distance_remaining < distance;
  }
  return false;
}
bool BaseAutonomousActor::SplineHandle::WaitForSplineDistanceRemaining(
    double distance) {
  ::aos::time::PhasedLoop phased_loop(
      ::std::chrono::milliseconds(5),
      base_autonomous_actor_->event_loop()->monotonic_now(),
      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (base_autonomous_actor_->ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    if (SplineDistanceRemaining(distance)) {
      return true;
    }
  }
}

void BaseAutonomousActor::LineFollowAtVelocity(double velocity, int hint) {
  auto drivetrain_message = drivetrain_goal_sender_.MakeMessage();
  drivetrain_message->controller_type = 3;
  // TODO(james): Currently the 4.0 is copied from the
  // line_follow_drivetrain.cc, but it is somewhat year-specific, so we should
  // factor it out in some way.
  drivetrain_message->throttle = velocity / 4.0;
  drivetrain_message.Send();
  auto target_hint = target_selector_hint_sender_.MakeMessage();
  target_hint->suggested_target = hint;
  target_hint.Send();
}

BaseAutonomousActor::SplineHandle BaseAutonomousActor::PlanSpline(
    const ::frc971::MultiSpline &spline, SplineDirection direction) {
  LOG(INFO, "Planning spline\n");

  int32_t spline_handle = (++spline_handle_) | ((getpid() & 0xFFFF) << 15);

  drivetrain_goal_fetcher_.Fetch();

  auto drivetrain_message =
      drivetrain_goal_sender_.MakeMessage();

  int controller_type = 2;
  if (drivetrain_goal_fetcher_.get()) {
    controller_type = drivetrain_goal_fetcher_->controller_type;
    drivetrain_message->throttle = drivetrain_goal_fetcher_->throttle;
  }
  drivetrain_message->controller_type = controller_type;

  drivetrain_message->spline = spline;
  drivetrain_message->spline.spline_idx = spline_handle;
  drivetrain_message->spline_handle = goal_spline_handle_;
  drivetrain_message->spline.drive_spline_backwards =
      direction == SplineDirection::kBackward;

  LOG_STRUCT(DEBUG, "dtg", *drivetrain_message);

  drivetrain_message.Send();

  return BaseAutonomousActor::SplineHandle(spline_handle, this);
}

bool BaseAutonomousActor::SplineHandle::IsPlanned() {
  base_autonomous_actor_->drivetrain_status_fetcher_.Fetch();
  LOG_STRUCT(DEBUG, "dts",
             *(base_autonomous_actor_->drivetrain_status_fetcher_.get()));
  if (base_autonomous_actor_->drivetrain_status_fetcher_.get() &&
      ((base_autonomous_actor_->drivetrain_status_fetcher_->trajectory_logging
                .planning_spline_idx == spline_handle_ &&
        base_autonomous_actor_->drivetrain_status_fetcher_->trajectory_logging
                .planning_state == 3) ||
       base_autonomous_actor_->drivetrain_status_fetcher_->trajectory_logging
               .current_spline_idx == spline_handle_)) {
    return true;
  }
  return false;
}

bool BaseAutonomousActor::SplineHandle::WaitForPlan() {
  ::aos::time::PhasedLoop phased_loop(
      ::std::chrono::milliseconds(5),
      base_autonomous_actor_->event_loop()->monotonic_now(),
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
  auto drivetrain_message =
      base_autonomous_actor_->drivetrain_goal_sender_.MakeMessage();
  drivetrain_message->controller_type = 2;

  LOG(INFO, "Starting spline\n");

  drivetrain_message->spline_handle = spline_handle_;
  base_autonomous_actor_->goal_spline_handle_ = spline_handle_;

  LOG_STRUCT(DEBUG, "dtg", *drivetrain_message);

  drivetrain_message.Send();
}

bool BaseAutonomousActor::SplineHandle::IsDone() {
  base_autonomous_actor_->drivetrain_status_fetcher_.Fetch();
  LOG_STRUCT(INFO, "dts",
             *(base_autonomous_actor_->drivetrain_status_fetcher_.get()));

  // We check that the spline we are waiting on is neither currently planning
  // nor executing (we check is_executed because it is possible to receive
  // status messages with is_executing false before the execution has started).
  // We check for planning so that the user can go straight from starting the
  // planner to executing without a WaitForPlan in between.
  if (base_autonomous_actor_->drivetrain_status_fetcher_.get() &&
      ((!base_autonomous_actor_->drivetrain_status_fetcher_->trajectory_logging
             .is_executed &&
        base_autonomous_actor_->drivetrain_status_fetcher_->trajectory_logging
                .current_spline_idx == spline_handle_) ||
       base_autonomous_actor_->drivetrain_status_fetcher_->trajectory_logging
               .planning_spline_idx == spline_handle_)) {
    return false;
  }
  return true;
}

bool BaseAutonomousActor::SplineHandle::WaitForDone() {
  ::aos::time::PhasedLoop phased_loop(
      ::std::chrono::milliseconds(5),
      base_autonomous_actor_->event_loop()->monotonic_now(),
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

}  // namespace autonomous
}  // namespace frc971
