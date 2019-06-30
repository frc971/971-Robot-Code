#include "y2014/actors/shoot_actor.h"

#include <functional>

#include "aos/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2014/constants.h"
#include "y2014/control_loops/claw/claw.q.h"
#include "y2014/control_loops/shooter/shooter.q.h"

namespace y2014 {
namespace actors {

constexpr double ShootActor::kOffsetRadians;
constexpr double ShootActor::kClawShootingSeparation;
constexpr double ShootActor::kClawShootingSeparationGoal;

ShootActor::ShootActor(::aos::EventLoop *event_loop)
    : ::aos::common::actions::ActorBase<actors::ShootActionQueueGroup>(
          event_loop, ".y2014.actors.shoot_action"),
      claw_goal_fetcher_(
          event_loop->MakeFetcher<::y2014::control_loops::ClawQueue::Goal>(
              ".y2014.control_loops.claw_queue.goal")),
      claw_status_fetcher_(
          event_loop->MakeFetcher<::y2014::control_loops::ClawQueue::Status>(
              ".y2014.control_loops.claw_queue.status")),
      claw_goal_sender_(
          event_loop->MakeSender<::y2014::control_loops::ClawQueue::Goal>(
              ".y2014.control_loops.claw_queue.goal")) {}

double ShootActor::SpeedToAngleOffset(double speed) {
  const constants::Values &values = constants::GetValues();
  // scale speed to a [0.0-1.0] on something close to the max
  return (speed / values.drivetrain_max_speed) * kOffsetRadians;
}

bool ShootActor::IntakeOff() {
  claw_goal_fetcher_.Fetch();
  if (!claw_goal_fetcher_.get()) {
    LOG(WARNING, "no claw goal\n");
    // If it doesn't have a goal, then the intake isn't on so we don't have to
    // turn it off.
    return true;
  } else {
    auto goal_message = claw_goal_sender_.MakeMessage();

    goal_message->bottom_angle = claw_goal_fetcher_->bottom_angle;
    goal_message->separation_angle = claw_goal_fetcher_->separation_angle;
    goal_message->intake = 0.0;
    goal_message->centering = 0.0;

    if (!goal_message.Send()) {
      LOG(WARNING, "sending claw goal failed\n");
      return false;
    }
  }
  return true;
}

bool ShootActor::RunAction(const double&) {
  InnerRunAction();

  // Now do our 'finally' block and make sure that we aren't requesting shots
  // continually.
  control_loops::shooter_queue.goal.FetchLatest();
  if (control_loops::shooter_queue.goal.get() == nullptr) {
    return true;
  }
  if (!control_loops::shooter_queue.goal.MakeWithBuilder()
           .shot_power(control_loops::shooter_queue.goal->shot_power)
           .shot_requested(false)
           .unload_requested(false)
           .load_requested(false)
           .Send()) {
    LOG(WARNING, "sending shooter goal failed\n");
    return false;
  }

  LOG(INFO, "finished\n");
  return true;
}

void ShootActor::InnerRunAction() {
  LOG(INFO, "Shooting at the current angle and power.\n");

  // wait for claw to be ready
  if (WaitUntil(::std::bind(&ShootActor::DoneSetupShot, this))) {
    return;
  }

  if (!IntakeOff()) return;

  // Make sure that we have the latest shooter status.
  control_loops::shooter_queue.status.FetchLatest();
  // Get the number of shots fired up to this point. This should not be updated
  // again for another few cycles.
  previous_shots_ = control_loops::shooter_queue.status->shots;
  // Shoot!
  if (!control_loops::shooter_queue.goal.MakeWithBuilder()
           .shot_power(control_loops::shooter_queue.goal->shot_power)
           .shot_requested(true)
           .unload_requested(false)
           .load_requested(false)
           .Send()) {
    LOG(WARNING, "sending shooter goal failed\n");
    return;
  }

  // wait for record of shot having been fired
  if (WaitUntil(::std::bind(&ShootActor::DoneShot, this))) return;

  if (!IntakeOff()) return;
}

bool ShootActor::ClawIsReady() {
  claw_goal_fetcher_.Fetch();

  bool ans = claw_status_fetcher_->zeroed &&
             (::std::abs(claw_status_fetcher_->bottom_velocity) < 0.5) &&
             (::std::abs(claw_status_fetcher_->bottom -
                         claw_goal_fetcher_->bottom_angle) < 0.10) &&
             (::std::abs(claw_status_fetcher_->separation -
                         claw_goal_fetcher_->separation_angle) < 0.4);
  LOG(DEBUG, "Claw is %sready zeroed %d bottom_velocity %f bottom %f sep %f\n",
      ans ? "" : "not ", claw_status_fetcher_->zeroed,
      ::std::abs(claw_status_fetcher_->bottom_velocity),
      ::std::abs(claw_status_fetcher_->bottom -
                 claw_goal_fetcher_->bottom_angle),
      ::std::abs(claw_status_fetcher_->separation -
                 claw_goal_fetcher_->separation_angle));
  return ans;
}

bool ShooterIsReady() {
  control_loops::shooter_queue.goal.FetchLatest();

  LOG(DEBUG, "Power error is %f - %f -> %f, ready %d\n",
      control_loops::shooter_queue.status->hard_stop_power,
      control_loops::shooter_queue.goal->shot_power,
      ::std::abs(control_loops::shooter_queue.status->hard_stop_power -
                 control_loops::shooter_queue.goal->shot_power),
      control_loops::shooter_queue.status->ready);
  return (::std::abs(control_loops::shooter_queue.status->hard_stop_power -
                     control_loops::shooter_queue.goal->shot_power) < 1.0) &&
         control_loops::shooter_queue.status->ready;
}

bool ShootActor::DoneSetupShot() {
  control_loops::shooter_queue.status.FetchAnother();
  claw_status_fetcher_.Fetch();
  // Make sure that both the shooter and claw have reached the necessary
  // states.
  if (ShooterIsReady() && ClawIsReady()) {
    LOG(INFO, "Claw and Shooter ready for shooting.\n");
    return true;
  }

  return false;
}

bool ShootActor::DonePreShotOpen() {
  claw_status_fetcher_.Fetch();
  if (claw_status_fetcher_->separation > kClawShootingSeparation) {
    LOG(INFO, "Opened up enough to shoot.\n");
    return true;
  }
  return false;
}

bool ShootActor::DoneShot() {
  control_loops::shooter_queue.status.FetchAnother();
  if (control_loops::shooter_queue.status->shots > previous_shots_) {
    LOG(INFO, "Shot succeeded!\n");
    return true;
  }
  return false;
}

}  // namespace actors
}  // namespace y2014
