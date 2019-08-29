#include "y2014/actors/shoot_actor.h"

#include <functional>

#include "aos/logging/logging.h"

#include "y2014/constants.h"
#include "y2014/control_loops/claw/claw_goal_generated.h"
#include "y2014/control_loops/claw/claw_status_generated.h"
#include "y2014/control_loops/shooter/shooter_goal_generated.h"
#include "y2014/control_loops/shooter/shooter_status_generated.h"

namespace y2014 {
namespace actors {

constexpr double ShootActor::kOffsetRadians;
constexpr double ShootActor::kClawShootingSeparation;
constexpr double ShootActor::kClawShootingSeparationGoal;

ShootActor::ShootActor(::aos::EventLoop *event_loop)
    : ::aos::common::actions::ActorBase<aos::common::actions::Goal>(
          event_loop, "/shoot_action"),
      claw_goal_fetcher_(
          event_loop->MakeFetcher<::y2014::control_loops::claw::Goal>("/claw")),
      claw_status_fetcher_(
          event_loop->MakeFetcher<::y2014::control_loops::claw::Status>(
              "/claw")),
      claw_goal_sender_(
          event_loop->MakeSender<::y2014::control_loops::claw::Goal>("/claw")),
      shooter_status_fetcher_(
          event_loop->MakeFetcher<::y2014::control_loops::shooter::Status>(
              "/shooter")),
      shooter_goal_fetcher_(
          event_loop->MakeFetcher<::y2014::control_loops::shooter::Goal>(
              "/shooter")),
      shooter_goal_sender_(
          event_loop->MakeSender<::y2014::control_loops::shooter::Goal>(
              "/shooter")) {}

double ShootActor::SpeedToAngleOffset(double speed) {
  const constants::Values &values = constants::GetValues();
  // scale speed to a [0.0-1.0] on something close to the max
  return (speed / values.drivetrain_max_speed) * kOffsetRadians;
}

bool ShootActor::IntakeOff() {
  claw_goal_fetcher_.Fetch();
  if (!claw_goal_fetcher_.get()) {
    AOS_LOG(WARNING, "no claw goal\n");
    // If it doesn't have a goal, then the intake isn't on so we don't have to
    // turn it off.
    return true;
  } else {
    auto builder = claw_goal_sender_.MakeBuilder();

    control_loops::claw::Goal::Builder claw_builder =
        builder.MakeBuilder<control_loops::claw::Goal>();

    claw_builder.add_bottom_angle(claw_goal_fetcher_->bottom_angle());
    claw_builder.add_separation_angle(claw_goal_fetcher_->separation_angle());
    claw_builder.add_intake(0.0);
    claw_builder.add_centering(0.0);

    if (!builder.Send(claw_builder.Finish())) {
      AOS_LOG(WARNING, "sending claw goal failed\n");
      return false;
    }
  }
  return true;
}

bool ShootActor::RunAction(const aos::common::actions::DoubleParam *) {
  InnerRunAction();

  // Now do our 'finally' block and make sure that we aren't requesting shots
  // continually.
  shooter_goal_fetcher_.Fetch();
  if (shooter_goal_fetcher_.get() == nullptr) {
    return true;
  }
  auto builder = shooter_goal_sender_.MakeBuilder();
  control_loops::shooter::Goal::Builder shooter_builder =
      builder.MakeBuilder<control_loops::shooter::Goal>();

  shooter_builder.add_shot_power(shooter_goal_fetcher_->shot_power());
  shooter_builder.add_shot_requested(false);
  shooter_builder.add_unload_requested(false);
  shooter_builder.add_load_requested(false);
  if (!builder.Send(shooter_builder.Finish())) {
    AOS_LOG(WARNING, "sending shooter goal failed\n");
    return false;
  }

  AOS_LOG(INFO, "finished\n");
  return true;
}

void ShootActor::InnerRunAction() {
  AOS_LOG(INFO, "Shooting at the current angle and power.\n");

  // wait for claw to be ready
  if (WaitUntil(::std::bind(&ShootActor::DoneSetupShot, this))) {
    return;
  }

  if (!IntakeOff()) return;

  // Make sure that we have the latest shooter status.
  shooter_status_fetcher_.Fetch();
  // Get the number of shots fired up to this point. This should not be updated
  // again for another few cycles.
  previous_shots_ = shooter_status_fetcher_->shots();
  // Shoot!
  shooter_goal_fetcher_.Fetch();
  {
    auto builder = shooter_goal_sender_.MakeBuilder();
    control_loops::shooter::Goal::Builder goal_builder =
        builder.MakeBuilder<control_loops::shooter::Goal>();
    goal_builder.add_shot_power(shooter_goal_fetcher_->shot_power());
    goal_builder.add_shot_requested(true);
    goal_builder.add_unload_requested(false);
    goal_builder.add_load_requested(false);
    if (!builder.Send(goal_builder.Finish())) {
      AOS_LOG(WARNING, "sending shooter goal failed\n");
      return;
    }
  }

  // wait for record of shot having been fired
  if (WaitUntil(::std::bind(&ShootActor::DoneShot, this))) return;

  if (!IntakeOff()) return;
}

bool ShootActor::ClawIsReady() {
  claw_goal_fetcher_.Fetch();

  bool ans = claw_status_fetcher_->zeroed() &&
             (::std::abs(claw_status_fetcher_->bottom_velocity()) < 0.5) &&
             (::std::abs(claw_status_fetcher_->bottom() -
                         claw_goal_fetcher_->bottom_angle()) < 0.10) &&
             (::std::abs(claw_status_fetcher_->separation() -
                         claw_goal_fetcher_->separation_angle()) < 0.4);
  AOS_LOG(DEBUG,
          "Claw is %sready zeroed %d bottom_velocity %f bottom %f sep %f\n",
          ans ? "" : "not ", claw_status_fetcher_->zeroed(),
          ::std::abs(claw_status_fetcher_->bottom_velocity()),
          ::std::abs(claw_status_fetcher_->bottom() -
                     claw_goal_fetcher_->bottom_angle()),
          ::std::abs(claw_status_fetcher_->separation() -
                     claw_goal_fetcher_->separation_angle()));
  return ans;
}

bool ShootActor::ShooterIsReady() {
  shooter_goal_fetcher_.Fetch();

  AOS_LOG(DEBUG, "Power error is %f - %f -> %f, ready %d\n",
          shooter_status_fetcher_->hard_stop_power(),
          shooter_goal_fetcher_->shot_power(),
          ::std::abs(shooter_status_fetcher_->hard_stop_power() -
                     shooter_goal_fetcher_->shot_power()),
          shooter_status_fetcher_->ready());
  return (::std::abs(shooter_status_fetcher_->hard_stop_power() -
                     shooter_goal_fetcher_->shot_power()) < 1.0) &&
         shooter_status_fetcher_->ready();
}

bool ShootActor::DoneSetupShot() {
  shooter_status_fetcher_.Fetch();
  claw_status_fetcher_.Fetch();
  // Make sure that both the shooter and claw have reached the necessary
  // states.
  if (ShooterIsReady() && ClawIsReady()) {
    AOS_LOG(INFO, "Claw and Shooter ready for shooting.\n");
    return true;
  }

  return false;
}

bool ShootActor::DonePreShotOpen() {
  claw_status_fetcher_.Fetch();
  if (claw_status_fetcher_->separation() > kClawShootingSeparation) {
    AOS_LOG(INFO, "Opened up enough to shoot.\n");
    return true;
  }
  return false;
}

bool ShootActor::DoneShot() {
  shooter_status_fetcher_.Fetch();
  if (shooter_status_fetcher_.get() &&
      shooter_status_fetcher_->shots() > previous_shots_) {
    AOS_LOG(INFO, "Shot succeeded!\n");
    return true;
  }
  return false;
}

}  // namespace actors
}  // namespace y2014
