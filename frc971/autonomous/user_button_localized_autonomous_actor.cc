#include "frc971/autonomous/user_button_localized_autonomous_actor.h"

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;
namespace drivetrain = frc971::control_loops::drivetrain;

namespace frc971 {
namespace autonomous {

UserButtonLocalizedAutonomousActor::UserButtonLocalizedAutonomousActor(
    ::aos::EventLoop *event_loop,
    const control_loops::drivetrain::DrivetrainConfig<double> &dt_config)
    : BaseAutonomousActor(event_loop, dt_config),
      robot_state_fetcher_(event_loop->MakeFetcher<aos::RobotState>("/aos")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")) {
  drivetrain_status_fetcher_.Fetch();
  replan_timer_ = event_loop->AddTimer([this]() { DoReplan(); });

  event_loop->OnRun([this, event_loop]() {
    replan_timer_->Schedule(event_loop->monotonic_now());
    button_poll_->Schedule(event_loop->monotonic_now(),
                           chrono::milliseconds(50));
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
        // Only kick the planning out by 2 seconds. If we end up enabled in
        // that second, then we will kick it out further based on the code
        // below.
        replan_timer_->Schedule(now + std::chrono::seconds(2));
      }
      if (joystick_state_fetcher_->enabled()) {
        if (!is_planned_) {
          // Only replan once we've been disabled for 5 seconds.
          replan_timer_->Schedule(now + std::chrono::seconds(5));
        }
      }
    }
  });
}

void UserButtonLocalizedAutonomousActor::DoReplan() {
  if (!drivetrain_status_fetcher_.Fetch()) {
    replan_timer_->Schedule(event_loop()->monotonic_now() + chrono::seconds(1));
    AOS_LOG(INFO, "Drivetrain not up, replanning in 1 second");
    return;
  }

  if (alliance_ == aos::Alliance::kInvalid) {
    return;
  }

  sent_starting_position_ = false;

  Replan();
}

void UserButtonLocalizedAutonomousActor::MaybeSendStartingPosition() {
  if (is_planned_ && user_indicated_safe_to_reset_ &&
      !sent_starting_position_) {
    CHECK(starting_position_);
    SendStartingPosition(starting_position_.value());
  }
}

void UserButtonLocalizedAutonomousActor::DoReset() {
  InitializeEncoders();
  ResetDrivetrain();

  joystick_state_fetcher_.Fetch();
  CHECK(joystick_state_fetcher_.get() != nullptr)
      << "Expect at least one JoystickState message before running auto...";
  alliance_ = joystick_state_fetcher_->alliance();

  Reset();
}

bool UserButtonLocalizedAutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  DoReset();

  AOS_LOG(INFO, "Params are %d\n", params->mode());

  if (!user_indicated_safe_to_reset_) {
    AOS_LOG(WARNING, "Didn't send starting position prior to starting auto.");
    CHECK(starting_position_);
    SendStartingPosition(starting_position_.value());
  }
  // Clear this so that we don't accidentally resend things as soon as we
  // replan later.
  user_indicated_safe_to_reset_ = false;
  is_planned_ = false;
  starting_position_.reset();

  return Run(params);
}

}  // namespace autonomous
}  // namespace frc971
