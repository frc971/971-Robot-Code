#include "y2022_bot3/actors/autonomous_actor.h"

#include <chrono>
#include <cinttypes>
#include <cmath>

#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "aos/util/math.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2022_bot3/actors/auto_splines.h"
#include "y2022_bot3/constants.h"
#include "y2022_bot3/control_loops/drivetrain/drivetrain_base.h"

DEFINE_bool(spline_auto, false, "If true, define a spline autonomous mode");

namespace y2022_bot3 {
namespace actors {

using ::aos::monotonic_clock;
using frc971::CreateProfileParameters;
using ::frc971::ProfileParametersT;
using frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::control_loops::drivetrain::LocalizerControl;

namespace chrono = ::std::chrono;

AutonomousActor::AutonomousActor(::aos::EventLoop *event_loop)
    : frc971::autonomous::BaseAutonomousActor(
          event_loop, control_loops::drivetrain::GetDrivetrainConfig()),
      localizer_control_sender_(
          event_loop->MakeSender<
              ::frc971::control_loops::drivetrain::LocalizerControl>(
              "/drivetrain")),
      superstructure_goal_sender_(
          event_loop->MakeSender<control_loops::superstructure::Goal>(
              "/superstructure")),
      superstructure_status_fetcher_(
          event_loop->MakeFetcher<control_loops::superstructure::Status>(
              "/superstructure")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      robot_state_fetcher_(event_loop->MakeFetcher<aos::RobotState>("/aos")),
      auto_splines_() {
  set_max_drivetrain_voltage(12.0);
  replan_timer_ = event_loop->AddTimer([this]() { Replan(); });
  event_loop->OnRun([this, event_loop]() {
    replan_timer_->Setup(event_loop->monotonic_now());
    button_poll_->Setup(event_loop->monotonic_now(), chrono::milliseconds(50));
  });

  button_poll_ = event_loop->AddTimer([this]() {
    const aos::monotonic_clock::time_point now =
        this->event_loop()->context().monotonic_event_time;
    if (robot_state_fetcher_.Fetch()) {
      if (robot_state_fetcher_->user_button()) {
        user_indicated_safe_to_reset_ = true;
        MaybeSendStartingPosition();
      }
    }
    if (joystick_state_fetcher_.Fetch()) {
      if (joystick_state_fetcher_->has_alliance() &&
          (joystick_state_fetcher_->alliance() != alliance_)) {
        alliance_ = joystick_state_fetcher_->alliance();
        is_planned_ = false;
        // Only kick the planning out by 2 seconds. If we end up enabled in that
        // second, then we will kick it out further based on the code below.
        replan_timer_->Setup(now + std::chrono::seconds(2));
      }
      if (joystick_state_fetcher_->enabled()) {
        if (!is_planned_) {
          // Only replan once we've been disabled for 5 seconds.
          replan_timer_->Setup(now + std::chrono::seconds(5));
        }
      }
    }
  });
}

void AutonomousActor::Replan() {
  LOG(INFO) << "Alliance " << static_cast<int>(alliance_);
  if (alliance_ == aos::Alliance::kInvalid) {
    return;
  }
  sent_starting_position_ = false;
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

void AutonomousActor::MaybeSendStartingPosition() {
  if (is_planned_ && user_indicated_safe_to_reset_ &&
      !sent_starting_position_) {
    CHECK(starting_position_);
    SendStartingPosition(starting_position_.value());
  }
}

void AutonomousActor::Reset() {
  InitializeEncoders();
  ResetDrivetrain();

  joystick_state_fetcher_.Fetch();
  CHECK(joystick_state_fetcher_.get() != nullptr)
      << "Expect at least one JoystickState message before running auto...";
  alliance_ = joystick_state_fetcher_->alliance();
}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  Reset();
  if (!user_indicated_safe_to_reset_) {
    AOS_LOG(WARNING, "Didn't send starting position prior to starting auto.");
    CHECK(starting_position_);
    SendStartingPosition(starting_position_.value());
  }
  // Clear this so that we don't accidentally resend things as soon as we replan
  // later.
  user_indicated_safe_to_reset_ = false;
  is_planned_ = false;
  starting_position_.reset();

  AOS_LOG(INFO, "Params are %d\n", params->mode());
  if (alliance_ == aos::Alliance::kInvalid) {
    AOS_LOG(INFO, "Aborting autonomous due to invalid alliance selection.");
    return false;
  }
  if (FLAGS_spline_auto) {
    SplineAuto();
  }

  return true;
}

void AutonomousActor::SendStartingPosition(const Eigen::Vector3d &start) {
  // Set up the starting position for the blue alliance.

  // TODO(james): Resetting the localizer breaks the left/right statespace
  // controller.  That is a bug, but we can fix that later by not resetting.
  auto builder = localizer_control_sender_.MakeBuilder();

  LocalizerControl::Builder localizer_control_builder =
      builder.MakeBuilder<LocalizerControl>();
  localizer_control_builder.add_x(start(0));
  localizer_control_builder.add_y(start(1));
  localizer_control_builder.add_theta(start(2));
  localizer_control_builder.add_theta_uncertainty(0.00001);
  LOG(INFO) << "User button pressed, x: " << start(0) << " y: " << start(1)
            << " theta: " << start(2);
  if (builder.Send(localizer_control_builder.Finish()) !=
      aos::RawSender::Error::kOk) {
    AOS_LOG(ERROR, "Failed to reset localizer.\n");
  }
}

void AutonomousActor::SplineAuto() {
  CHECK(test_spline_);

  if (!test_spline_->WaitForPlan()) return;
  test_spline_->Start();

  if (!test_spline_->WaitForSplineDistanceRemaining(0.02)) return;
}

void AutonomousActor::SendSuperstructureGoal() {
  auto builder = superstructure_goal_sender_.MakeBuilder();
  superstructure::Goal::Builder superstructure_builder = builder.MakeBuilder<superstructure::Goal>();
  if (builder.Send(superstructure_builder.Finish()) !=
      aos::RawSender::Error::kOk) {
    AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
  }
}

}  // namespace actors
}  // namespace y2022_bot3
