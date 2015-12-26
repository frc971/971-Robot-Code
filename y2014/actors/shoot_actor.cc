#include "y2014/actors/shoot_actor.h"

#include <functional>

#include "aos/common/logging/logging.h"

#include "y2014/control_loops/shooter/shooter.q.h"
#include "y2014/control_loops/claw/claw.q.h"
#include "y2014/constants.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"

namespace y2014 {
namespace actors {
namespace {

bool IntakeOff() {
  control_loops::claw_queue.goal.FetchLatest();
  if (!control_loops::claw_queue.goal.get()) {
    LOG(WARNING, "no claw goal\n");
    // If it doesn't have a goal, then the intake isn't on so we don't have to
    // turn it off.
    return true;
  } else {
    if (!control_loops::claw_queue.goal.MakeWithBuilder()
             .bottom_angle(control_loops::claw_queue.goal->bottom_angle)
             .separation_angle(control_loops::claw_queue.goal->separation_angle)
             .intake(0.0)
             .centering(0.0)
             .Send()) {
      LOG(WARNING, "sending claw goal failed\n");
      return false;
    }
  }
  return true;
}

}  // namespace

constexpr double ShootActor::kOffsetRadians;
constexpr double ShootActor::kClawShootingSeparation;
constexpr double ShootActor::kClawShootingSeparationGoal;

ShootActor::ShootActor(actors::ShootActionQueueGroup* s)
    : ::aos::common::actions::ActorBase<actors::ShootActionQueueGroup>(s) {}

double ShootActor::SpeedToAngleOffset(double speed) {
  const constants::Values& values = constants::GetValues();
  // scale speed to a [0.0-1.0] on something close to the max
  return (speed / values.drivetrain_max_speed) * kOffsetRadians;
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

bool ClawIsReady() {
  control_loops::claw_queue.goal.FetchLatest();

  bool ans =
      control_loops::claw_queue.status->zeroed &&
      (::std::abs(control_loops::claw_queue.status->bottom_velocity) < 0.5) &&
      (::std::abs(control_loops::claw_queue.status->bottom -
                  control_loops::claw_queue.goal->bottom_angle) < 0.10) &&
      (::std::abs(control_loops::claw_queue.status->separation -
                  control_loops::claw_queue.goal->separation_angle) < 0.4);
  LOG(DEBUG, "Claw is %sready zeroed %d bottom_velocity %f bottom %f sep %f\n",
      ans ? "" : "not ", control_loops::claw_queue.status->zeroed,
      ::std::abs(control_loops::claw_queue.status->bottom_velocity),
      ::std::abs(control_loops::claw_queue.status->bottom -
                 control_loops::claw_queue.goal->bottom_angle),
      ::std::abs(control_loops::claw_queue.status->separation -
                 control_loops::claw_queue.goal->separation_angle));
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
  control_loops::claw_queue.status.FetchAnother();
  // Make sure that both the shooter and claw have reached the necessary
  // states.
  if (ShooterIsReady() && ClawIsReady()) {
    LOG(INFO, "Claw and Shooter ready for shooting.\n");
    return true;
  }

  return false;
}

bool ShootActor::DonePreShotOpen() {
  control_loops::claw_queue.status.FetchAnother();
  if (control_loops::claw_queue.status->separation > kClawShootingSeparation) {
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

::std::unique_ptr<ShootAction> MakeShootAction() {
  return ::std::unique_ptr<ShootAction>(
      new ShootAction(&::y2014::actors::shoot_action, 0.0));
}

}  // namespace actors
}  // namespace y2014
