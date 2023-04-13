#include "frc971/autonomous/base_autonomous_actor.h"

#include <chrono>
#include <cinttypes>
#include <cmath>

#include "aos/logging/logging.h"
#include "aos/util/math.h"
#include "aos/util/phased_loop.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/drivetrain/spline.h"
#include "y2019/control_loops/drivetrain/target_selector_generated.h"

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;
namespace drivetrain = frc971::control_loops::drivetrain;

namespace frc971 {
namespace autonomous {

BaseAutonomousActor::BaseAutonomousActor(
    ::aos::EventLoop *event_loop,
    const control_loops::drivetrain::DrivetrainConfig<double> &dt_config)
    : aos::common::actions::ActorBase<Goal>(event_loop, "/autonomous"),
      dt_config_(dt_config),
      initial_drivetrain_({0.0, 0.0}),
      target_selector_hint_sender_(
          event_loop->TryMakeSender<
              ::y2019::control_loops::drivetrain::TargetSelectorHint>(
              "/drivetrain")),
      drivetrain_goal_sender_(
          event_loop->MakeSender<drivetrain::Goal>("/drivetrain")),
      spline_goal_sender_(
          event_loop->MakeSender<drivetrain::SplineGoal>("/drivetrain")),
      drivetrain_status_fetcher_(
          event_loop->MakeFetcher<drivetrain::Status>("/drivetrain")),
      drivetrain_goal_fetcher_(
          event_loop->MakeFetcher<drivetrain::Goal>("/drivetrain")) {
  event_loop->SetRuntimeRealtimePriority(29);
}

void BaseAutonomousActor::ApplyThrottle(double throttle) {
  goal_spline_handle_ = 0;

  auto builder = drivetrain_goal_sender_.MakeBuilder();

  drivetrain::Goal::Builder goal_builder =
      builder.MakeBuilder<drivetrain::Goal>();
  goal_builder.add_controller_type(drivetrain::ControllerType::POLYDRIVE);
  goal_builder.add_highgear(true);
  goal_builder.add_wheel(0.0);
  goal_builder.add_throttle(throttle);
  builder.CheckOk(builder.Send(goal_builder.Finish()));
}

void BaseAutonomousActor::ResetDrivetrain() {
  AOS_LOG(INFO, "resetting the drivetrain\n");
  max_drivetrain_voltage_ = 12.0;
  goal_spline_handle_ = 0;

  auto builder = drivetrain_goal_sender_.MakeBuilder();

  drivetrain::Goal::Builder goal_builder =
      builder.MakeBuilder<drivetrain::Goal>();
  goal_builder.add_controller_type(drivetrain::ControllerType::POLYDRIVE);
  goal_builder.add_highgear(true);
  goal_builder.add_wheel(0.0);
  goal_builder.add_throttle(0.0);
  goal_builder.add_left_goal(initial_drivetrain_.left);
  goal_builder.add_right_goal(initial_drivetrain_.right);
  goal_builder.add_max_ss_voltage(max_drivetrain_voltage_);
  builder.CheckOk(builder.Send(goal_builder.Finish()));
}

void BaseAutonomousActor::InitializeEncoders() {
  // Spin until we get a message.
  WaitUntil([this]() { return drivetrain_status_fetcher_.Fetch(); });

  initial_drivetrain_.left =
      drivetrain_status_fetcher_->estimated_left_position();
  initial_drivetrain_.right =
      drivetrain_status_fetcher_->estimated_right_position();
}

void BaseAutonomousActor::StartDrive(double distance, double angle,
                                     ProfileParametersT linear,
                                     ProfileParametersT angular) {
  AOS_LOG(INFO, "Driving distance %f, angle %f\n", distance, angle);
  {
    const double dangle = angle * dt_config_.robot_radius;
    initial_drivetrain_.left += distance - dangle;
    initial_drivetrain_.right += distance + dangle;
  }

  auto builder = drivetrain_goal_sender_.MakeBuilder();

  auto linear_offset = ProfileParameters::Pack(*builder.fbb(), &linear);
  auto angular_offset = ProfileParameters::Pack(*builder.fbb(), &angular);

  drivetrain::Goal::Builder goal_builder =
      builder.MakeBuilder<drivetrain::Goal>();

  goal_builder.add_controller_type(drivetrain::ControllerType::MOTION_PROFILE);
  goal_builder.add_highgear(true);
  goal_builder.add_wheel(0.0);
  goal_builder.add_throttle(0.0);
  goal_builder.add_left_goal(initial_drivetrain_.left);
  goal_builder.add_right_goal(initial_drivetrain_.right);
  goal_builder.add_max_ss_voltage(max_drivetrain_voltage_);
  goal_builder.add_linear(linear_offset);
  goal_builder.add_angular(angular_offset);

  builder.CheckOk(builder.Send(goal_builder.Finish()));
}

void BaseAutonomousActor::WaitUntilDoneOrCanceled(
    ::std::unique_ptr<aos::common::actions::Action> action) {
  if (!action) {
    AOS_LOG(ERROR, "No action, not waiting\n");
    return;
  }

  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      ActorBase::kLoopOffset);
  while (true) {
    // Poll the running bit and see if we should cancel.
    phased_loop.SleepUntilNext();
    if (!action->Running() || ShouldCancel()) {
      return;
    }
  }
}

bool BaseAutonomousActor::WaitForDriveDone() {
  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      ActorBase::kLoopOffset);

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
    if (::std::abs(drivetrain_status_fetcher_->profiled_left_position_goal() -
                   initial_drivetrain_.left) < kProfileTolerance &&
        ::std::abs(drivetrain_status_fetcher_->profiled_right_position_goal() -
                   initial_drivetrain_.right) < kProfileTolerance &&
        ::std::abs(drivetrain_status_fetcher_->estimated_left_position() -
                   initial_drivetrain_.left) < kPositionTolerance &&
        ::std::abs(drivetrain_status_fetcher_->estimated_right_position() -
                   initial_drivetrain_.right) < kPositionTolerance &&
        ::std::abs(drivetrain_status_fetcher_->estimated_left_velocity()) <
            kVelocityTolerance &&
        ::std::abs(drivetrain_status_fetcher_->estimated_right_velocity()) <
            kVelocityTolerance) {
      AOS_LOG(INFO, "Finished drive\n");
      return true;
    }
  }
  return false;
}

double BaseAutonomousActor::X() {
  drivetrain_status_fetcher_.Fetch();
  CHECK(drivetrain_status_fetcher_.get());
  return drivetrain_status_fetcher_->x();
}

double BaseAutonomousActor::Y() {
  drivetrain_status_fetcher_.Fetch();
  CHECK(drivetrain_status_fetcher_.get());
  return drivetrain_status_fetcher_->y();
}

double BaseAutonomousActor::Theta() {
  drivetrain_status_fetcher_.Fetch();
  CHECK(drivetrain_status_fetcher_.get());
  return drivetrain_status_fetcher_->theta();
}

bool BaseAutonomousActor::WaitForAboveAngle(double angle) {
  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      ActorBase::kLoopOffset);
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
      if (drivetrain_status_fetcher_->ground_angle() > angle) {
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForBelowAngle(double angle) {
  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      ActorBase::kLoopOffset);
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
      if (drivetrain_status_fetcher_->ground_angle() < angle) {
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForMaxBy(double angle) {
  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      ActorBase::kLoopOffset);
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
      if (drivetrain_status_fetcher_->ground_angle() > max_angle) {
        max_angle = drivetrain_status_fetcher_->ground_angle();
      }
      if (drivetrain_status_fetcher_->ground_angle() < max_angle - angle) {
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForDriveNear(double distance, double angle) {
  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      ActorBase::kLoopOffset);
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
           drivetrain_status_fetcher_->profiled_left_position_goal());
      const double right_profile_error =
          (initial_drivetrain_.right -
           drivetrain_status_fetcher_->profiled_right_position_goal());

      const double left_error =
          (initial_drivetrain_.left -
           drivetrain_status_fetcher_->estimated_left_position());
      const double right_error =
          (initial_drivetrain_.right -
           drivetrain_status_fetcher_->estimated_right_position());

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
        AOS_LOG(INFO, "Drive finished first\n");
        printed_first = true;
      } else if (!drive_has_been_close && turn_has_been_close &&
                 !printed_first) {
        AOS_LOG(INFO, "Turn finished first\n");
        printed_first = true;
      }

      if (drive_close && turn_close) {
        AOS_LOG(INFO, "Closer than %f < %f distance, %f < %f angle\n",
                distance_to_go, distance, angle_to_go, angle);
        return true;
      }
    }
  }
}

bool BaseAutonomousActor::WaitForDriveProfileNear(double tolerance) {
  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      ActorBase::kLoopOffset);
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_status_fetcher_.Fetch();

    const Eigen::Matrix<double, 7, 1> current_error =
        (Eigen::Matrix<double, 7, 1>()
             << initial_drivetrain_.left -
                    drivetrain_status_fetcher_->profiled_left_position_goal(),
         0.0,
         initial_drivetrain_.right -
             drivetrain_status_fetcher_->profiled_right_position_goal(),
         0.0, 0.0, 0.0, 0.0)
            .finished();
    const Eigen::Matrix<double, 2, 1> linear_error =
        dt_config_.LeftRightToLinear(current_error);

    if (drivetrain_status_fetcher_.get()) {
      if (::std::abs(linear_error(0)) < tolerance) {
        AOS_LOG(INFO, "Finished drive\n");
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
  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      ActorBase::kLoopOffset);
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_status_fetcher_.Fetch();

    const Eigen::Matrix<double, 7, 1> current_error =
        (Eigen::Matrix<double, 7, 1>()
             << initial_drivetrain_.left -
                    drivetrain_status_fetcher_->profiled_left_position_goal(),
         0.0,
         initial_drivetrain_.right -
             drivetrain_status_fetcher_->profiled_right_position_goal(),
         0.0, 0.0, 0.0, 0.0)
            .finished();
    const Eigen::Matrix<double, 2, 1> angular_error =
        dt_config_.LeftRightToAngular(current_error);

    if (drivetrain_status_fetcher_.get()) {
      if (::std::abs(angular_error(0)) < tolerance) {
        AOS_LOG(INFO, "Finished turn\n");
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
         drivetrain_status_fetcher_->estimated_left_position());
    const double right_error =
        (initial_drivetrain_.right -
         drivetrain_status_fetcher_->estimated_right_position());

    return (left_error + right_error) / 2.0;
  } else {
    return 0;
  }
}

bool BaseAutonomousActor::SplineHandle::SplineDistanceRemaining(
    double distance) {
  base_autonomous_actor_->drivetrain_status_fetcher_.Fetch();
  if (base_autonomous_actor_->drivetrain_status_fetcher_.get()) {
    // Confirm that:
    // (a) The spline has started executiong (is_executing remains true even
    //     when we reach the end of the spline).
    // (b) The spline that we are executing is the correct one.
    // (c) There is less than distance distance remaining.
    if (base_autonomous_actor_->drivetrain_status_fetcher_->trajectory_logging()
            ->goal_spline_handle() != spline_handle_) {
      // Never done if we aren't the active spline.
      return false;
    }

    if (base_autonomous_actor_->drivetrain_status_fetcher_->trajectory_logging()
            ->is_executed()) {
      return true;
    }
    return base_autonomous_actor_->drivetrain_status_fetcher_
               ->trajectory_logging()
               ->is_executing() &&
           base_autonomous_actor_->drivetrain_status_fetcher_
                   ->trajectory_logging()
                   ->distance_remaining() < distance;
  }
  return false;
}

bool BaseAutonomousActor::SplineHandle::SplineDistanceTraveled(
    double distance) {
  base_autonomous_actor_->drivetrain_status_fetcher_.Fetch();
  if (base_autonomous_actor_->drivetrain_status_fetcher_.get()) {
    // Confirm that:
    // (a) The spline has started executiong (is_executing remains true even
    //     when we reach the end of the spline).
    // (b) The spline that we are executing is the correct one.
    // (c) There is less than distance distance remaining.
    if (base_autonomous_actor_->drivetrain_status_fetcher_->trajectory_logging()
            ->goal_spline_handle() != spline_handle_) {
      // Never done if we aren't the active spline.
      return false;
    }

    if (base_autonomous_actor_->drivetrain_status_fetcher_->trajectory_logging()
            ->is_executed()) {
      return true;
    }
    return base_autonomous_actor_->drivetrain_status_fetcher_
               ->trajectory_logging()
               ->is_executing() &&
           base_autonomous_actor_->drivetrain_status_fetcher_
                   ->trajectory_logging()
                   ->distance_traveled() > distance;
  }
  return false;
}

bool BaseAutonomousActor::SplineHandle::WaitForSplineDistanceRemaining(
    double distance) {
  ::aos::time::PhasedLoop phased_loop(
      frc971::controls::kLoopFrequency,
      base_autonomous_actor_->event_loop()->monotonic_now(),
      ActorBase::kLoopOffset);
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

bool BaseAutonomousActor::SplineHandle::WaitForSplineDistanceTraveled(
    double distance) {
  ::aos::time::PhasedLoop phased_loop(
      frc971::controls::kLoopFrequency,
      base_autonomous_actor_->event_loop()->monotonic_now(),
      ActorBase::kLoopOffset);
  while (true) {
    if (base_autonomous_actor_->ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    if (SplineDistanceTraveled(distance)) {
      return true;
    }
  }
}

void BaseAutonomousActor::LineFollowAtVelocity(
    double velocity, y2019::control_loops::drivetrain::SelectionHint hint) {
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();
    drivetrain::Goal::Builder goal_builder =
        builder.MakeBuilder<drivetrain::Goal>();
    goal_builder.add_controller_type(
        drivetrain::ControllerType::SPLINE_FOLLOWER);
    // TODO(james): Currently the 4.0 is copied from the
    // line_follow_drivetrain.cc, but it is somewhat year-specific, so we should
    // factor it out in some way.
    goal_builder.add_throttle(velocity / 4.0);
    builder.CheckOk(builder.Send(goal_builder.Finish()));
  }

  if (target_selector_hint_sender_) {
    // TODO(james): 2019? Seriously?
    auto builder = target_selector_hint_sender_.MakeBuilder();
    ::y2019::control_loops::drivetrain::TargetSelectorHint::Builder
        target_hint_builder = builder.MakeBuilder<
            ::y2019::control_loops::drivetrain::TargetSelectorHint>();

    target_hint_builder.add_suggested_target(hint);
    builder.CheckOk(builder.Send(target_hint_builder.Finish()));
  }
}

BaseAutonomousActor::SplineHandle BaseAutonomousActor::PlanSpline(
    std::function<flatbuffers::Offset<frc971::MultiSpline>(
        aos::Sender<frc971::control_loops::drivetrain::SplineGoal>::Builder
            *builder)> &&multispline_builder,
    SplineDirection direction) {
  AOS_LOG(INFO, "Planning spline\n");

  int32_t spline_handle = (++spline_handle_) | ((getpid() & 0xFFFF) << 15);

  drivetrain_goal_fetcher_.Fetch();

  auto builder = spline_goal_sender_.MakeBuilder();

  flatbuffers::Offset<frc971::MultiSpline> multispline_offset =
      multispline_builder(&builder);

  drivetrain::SplineGoal::Builder spline_builder =
      builder.MakeBuilder<drivetrain::SplineGoal>();
  spline_builder.add_spline_idx(spline_handle);
  spline_builder.add_drive_spline_backwards(direction ==
                                            SplineDirection::kBackward);
  spline_builder.add_spline(multispline_offset);

  // Calculate the starting position and yaw.
  Eigen::Vector3d start;
  {
    const frc971::MultiSpline *const spline =
        flatbuffers::GetTemporaryPointer(*builder.fbb(), multispline_offset);

    start(0) = spline->spline_x()->Get(0);
    start(1) = spline->spline_y()->Get(0);

    Eigen::Matrix<double, 2, 6> control_points;
    for (size_t ii = 0; ii < 6; ++ii) {
      control_points(0, ii) = spline->spline_x()->Get(ii);
      control_points(1, ii) = spline->spline_y()->Get(ii);
    }

    frc971::control_loops::drivetrain::Spline spline_object(control_points);
    start(2) = spline_object.Theta(0);
    if (direction == SplineDirection::kBackward) {
      start(2) = aos::math::NormalizeAngle(start(2) + M_PI);
    }
  }

  builder.CheckOk(builder.Send(spline_builder.Finish()));

  return BaseAutonomousActor::SplineHandle(spline_handle, this, start);
}

bool BaseAutonomousActor::SplineHandle::IsPlanned() {
  base_autonomous_actor_->drivetrain_status_fetcher_.Fetch();
  VLOG(1) << aos::FlatbufferToJson(
      base_autonomous_actor_->drivetrain_status_fetcher_.get());

  if (base_autonomous_actor_->drivetrain_status_fetcher_.get() &&
      base_autonomous_actor_->drivetrain_status_fetcher_.get()
          ->has_trajectory_logging() &&
      base_autonomous_actor_->drivetrain_status_fetcher_.get()
          ->trajectory_logging()
          ->has_available_splines()) {
    const flatbuffers::Vector<int> *splines =
        base_autonomous_actor_->drivetrain_status_fetcher_.get()
            ->trajectory_logging()
            ->available_splines();

    return std::find(splines->begin(), splines->end(), spline_handle_) !=
           splines->end();
  }
  return false;
}

bool BaseAutonomousActor::SplineHandle::WaitForPlan() {
  ::aos::time::PhasedLoop phased_loop(
      frc971::controls::kLoopFrequency,
      base_autonomous_actor_->event_loop()->monotonic_now(),
      ActorBase::kLoopOffset);
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
  auto builder = base_autonomous_actor_->drivetrain_goal_sender_.MakeBuilder();
  drivetrain::Goal::Builder goal_builder =
      builder.MakeBuilder<drivetrain::Goal>();
  goal_builder.add_controller_type(drivetrain::ControllerType::SPLINE_FOLLOWER);

  AOS_LOG(INFO, "Starting spline\n");

  goal_builder.add_spline_handle(spline_handle_);
  base_autonomous_actor_->goal_spline_handle_ = spline_handle_;

  builder.CheckOk(builder.Send(goal_builder.Finish()));
}

bool BaseAutonomousActor::SplineHandle::IsDone() {
  base_autonomous_actor_->drivetrain_status_fetcher_.Fetch();

  // We check that the spline we are waiting on is neither currently planning
  // nor executing (we check is_executed because it is possible to receive
  // status messages with is_executing false before the execution has started).
  // We check for planning so that the user can go straight from starting the
  // planner to executing without a WaitForPlan in between.
  if (base_autonomous_actor_->drivetrain_status_fetcher_.get() &&
      ((!base_autonomous_actor_->drivetrain_status_fetcher_
             ->trajectory_logging()
             ->is_executed() &&
        base_autonomous_actor_->drivetrain_status_fetcher_->trajectory_logging()
                ->current_spline_idx() == spline_handle_) ||
       IsPlanned())) {
    return false;
  }
  return true;
}

bool BaseAutonomousActor::SplineHandle::WaitForDone() {
  ::aos::time::PhasedLoop phased_loop(
      frc971::controls::kLoopFrequency,
      base_autonomous_actor_->event_loop()->monotonic_now(),
      ActorBase::kLoopOffset);
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
