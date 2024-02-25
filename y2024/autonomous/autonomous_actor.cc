#include "y2024/autonomous/autonomous_actor.h"

#include <chrono>
#include <cinttypes>
#include <cmath>

#include "aos/logging/logging.h"
#include "aos/util/math.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2024/autonomous/auto_splines.h"
#include "y2024/constants.h"
#include "y2024/control_loops/drivetrain/drivetrain_base.h"

DEFINE_bool(spline_auto, false, "Run simple test S-spline auto mode.");

namespace y2024::autonomous {

using ::frc971::ProfileParametersT;

ProfileParametersT MakeProfileParameters(float max_velocity,
                                         float max_acceleration) {
  ProfileParametersT result;
  result.max_velocity = max_velocity;
  result.max_acceleration = max_acceleration;
  return result;
}

using ::aos::monotonic_clock;
using frc971::CreateProfileParameters;
using ::frc971::ProfileParametersT;
using frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::control_loops::drivetrain::LocalizerControl;
namespace chrono = ::std::chrono;

AutonomousActor::AutonomousActor(::aos::EventLoop *event_loop)
    : frc971::autonomous::UserButtonLocalizedAutonomousActor(
          event_loop,
          control_loops::drivetrain::GetDrivetrainConfig(event_loop)),
      localizer_control_sender_(
          event_loop->MakeSender<
              ::frc971::control_loops::drivetrain::LocalizerControl>(
              "/drivetrain")),
      superstructure_goal_sender_(
          event_loop
              ->MakeSender<::y2024::control_loops::superstructure::GoalStatic>(
                  "/superstructure")),
      superstructure_status_fetcher_(
          event_loop
              ->MakeFetcher<::y2024::control_loops::superstructure::Status>(
                  "/superstructure")),
      auto_splines_() {}

void AutonomousActor::Replan() {
  if (FLAGS_spline_auto) {
    test_spline_ =
        PlanSpline(std::bind(&AutonomousSplines::TestSpline, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kForward);

    starting_position_ = test_spline_->starting_position();
  }

  is_planned_ = true;

  MaybeSendStartingPosition();
}

bool AutonomousActor::Run(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  AOS_LOG(INFO, "Params are %d\n", params->mode());
  if (alliance_ == aos::Alliance::kInvalid) {
    AOS_LOG(INFO, "Aborting autonomous due to invalid alliance selection.");
    return false;
  }
  if (FLAGS_spline_auto) {
    SplineAuto();
  } else {
    AOS_LOG(WARNING, "No auto mode selected.");
  }
  return true;
}

void AutonomousActor::SendStartingPosition(const Eigen::Vector3d &start) {
  // Set up the starting position for the blue alliance.

  auto builder = localizer_control_sender_.MakeBuilder();

  LocalizerControl::Builder localizer_control_builder =
      builder.MakeBuilder<LocalizerControl>();
  localizer_control_builder.add_x(start(0));
  localizer_control_builder.add_y(start(1));
  localizer_control_builder.add_theta(start(2));
  localizer_control_builder.add_theta_uncertainty(0.00001);
  AOS_LOG(INFO, "User button pressed, x: %f y: %f theta: %f", start(0),
          start(1), start(2));
  if (builder.Send(localizer_control_builder.Finish()) !=
      aos::RawSender::Error::kOk) {
    AOS_LOG(ERROR, "Failed to reset localizer.\n");
  }
}

void AutonomousActor::Reset() {
  set_intake_goal(control_loops::superstructure::IntakeGoal::NONE);
  set_note_goal(control_loops::superstructure::NoteGoal::NONE);
  set_auto_aim(false);
  set_fire(false);
  set_preloaded(false);
  SendSuperstructureGoal();
}

void AutonomousActor::SplineAuto() {
  CHECK(test_spline_);

  if (!test_spline_->WaitForPlan()) return;
  test_spline_->Start();

  if (!test_spline_->WaitForSplineDistanceRemaining(0.02)) return;
}

void AutonomousActor::SendSuperstructureGoal() {
  aos::Sender<control_loops::superstructure::GoalStatic>::StaticBuilder
      goal_builder = superstructure_goal_sender_.MakeStaticBuilder();

  goal_builder->set_intake_goal(intake_goal_);
  goal_builder->set_note_goal(note_goal_);
  goal_builder->set_fire(fire_);

  control_loops::superstructure::ShooterGoalStatic *shooter_goal =
      goal_builder->add_shooter_goal();

  shooter_goal->set_auto_aim(auto_aim_);
  shooter_goal->set_preloaded(preloaded_);

  goal_builder.CheckOk(goal_builder.Send());
}

void AutonomousActor::Intake() {
  set_intake_goal(control_loops::superstructure::IntakeGoal::INTAKE);
  set_note_goal(control_loops::superstructure::NoteGoal::CATAPULT);
  SendSuperstructureGoal();
}

void AutonomousActor::Aim() {
  set_auto_aim(true);
  SendSuperstructureGoal();
}

void AutonomousActor::Shoot() {
  set_fire(true);
  SendSuperstructureGoal();
}

[[nodiscard]] bool AutonomousActor::WaitForPreloaded() {
  set_preloaded(true);
  SendSuperstructureGoal();

  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      aos::common::actions::kLoopOffset);

  bool loaded = false;
  while (!loaded) {
    if (ShouldCancel()) {
      return false;
    }

    phased_loop.SleepUntilNext();
    superstructure_status_fetcher_.Fetch();
    CHECK(superstructure_status_fetcher_.get() != nullptr);

    loaded = (superstructure_status_fetcher_->shooter()->catapult_state() ==
              control_loops::superstructure::CatapultState::LOADED);
  }

  set_preloaded(false);
  SendSuperstructureGoal();

  return true;
}

}  // namespace y2024::autonomous
