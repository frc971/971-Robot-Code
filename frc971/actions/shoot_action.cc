#include <functional>

#include "aos/common/control_loop/Timing.h"
#include "aos/common/logging/logging.h"

#include "frc971/actions/shoot_action.h"
#include "frc971/control_loops/shooter/shooter.q.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/constants.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"

namespace frc971 {
namespace actions {

ShootAction::ShootAction(actions::ShootActionQueueGroup* s)
    : actions::ActionBase<actions::ShootActionQueueGroup>(s) {}

double ShootAction::SpeedToAngleOffset(double speed) {
  const frc971::constants::Values& values = frc971::constants::GetValues();
  // scale speed to a [0.0-1.0] on something close to the max
  return (speed / values.drivetrain_max_speed) * ShootAction::kOffsetRadians;
}
void ShootAction::RunAction() {
  InnerRunAction();

  // Now do our 'finally' block and make sure that we aren't requesting shots
  // continually.
  control_loops::shooter_queue_group.goal.FetchLatest();
  if (control_loops::shooter_queue_group.goal.get() == nullptr) {
    return;
  }
  if (!control_loops::shooter_queue_group.goal.MakeWithBuilder()
           .shot_power(control_loops::shooter_queue_group.goal->shot_power)
           .shot_requested(false)
           .unload_requested(false)
           .load_requested(false)
           .Send()) {
    LOG(WARNING, "sending shooter goal failed\n");
    return;
  }
}

void ShootAction::InnerRunAction() {
  LOG(INFO, "Shooting at the original angle and power.\n");

  // wait for claw to be ready
  if (WaitUntil(::std::bind(&ShootAction::DoneSetupShot, this))) return;

  // Turn the intake off.
  control_loops::claw_queue_group.goal.FetchLatest();

  if (!control_loops::claw_queue_group.goal.MakeWithBuilder()
           .bottom_angle(control_loops::claw_queue_group.goal->bottom_angle)
           .separation_angle(
                control_loops::claw_queue_group.goal->separation_angle)
           .intake(0.0)
           .centering(0.0)
           .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // Make sure that we have the latest shooter status.
  control_loops::shooter_queue_group.status.FetchLatest();
  // Get the number of shots fired up to this point. This should not be updated
  // again for another few cycles.
  previous_shots_ = control_loops::shooter_queue_group.status->shots;
  // Shoot!
  if (!control_loops::shooter_queue_group.goal.MakeWithBuilder()
           .shot_power(control_loops::shooter_queue_group.goal->shot_power)
           .shot_requested(true)
           .unload_requested(false)
           .load_requested(false)
           .Send()) {
    LOG(WARNING, "sending shooter goal failed\n");
    return;
  }

  // wait for record of shot having been fired
  if (WaitUntil(::std::bind(&ShootAction::DoneShot, this))) return;

  // Turn the intake off.
  control_loops::claw_queue_group.goal.FetchLatest();

  if (!control_loops::claw_queue_group.goal.MakeWithBuilder()
           .bottom_angle(control_loops::claw_queue_group.goal->bottom_angle)
           .separation_angle(
                control_loops::claw_queue_group.goal->separation_angle)
           .intake(0.0)
           .centering(0.0)
           .Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }
}

bool ClawIsReady() {
  if (!control_loops::claw_queue_group.goal.FetchLatest()) {
    control_loops::claw_queue_group.goal.FetchLatest();
  }

  bool ans =
      control_loops::claw_queue_group.status->zeroed &&
      (::std::abs(control_loops::claw_queue_group.status->bottom_velocity) <
       0.5) &&
      (::std::abs(control_loops::claw_queue_group.status->bottom -
                  control_loops::claw_queue_group.goal->bottom_angle) < 0.10) &&
      (::std::abs(control_loops::claw_queue_group.status->separation -
                  control_loops::claw_queue_group.goal->separation_angle) <
       0.4);
  if (!ans) {
    LOG(INFO,
        "Claw is ready %d zeroed %d bottom_velocity %f bottom %f sep %f\n", ans,
        control_loops::claw_queue_group.status->zeroed,
        ::std::abs(control_loops::claw_queue_group.status->bottom_velocity),
        ::std::abs(control_loops::claw_queue_group.status->bottom -
                   control_loops::claw_queue_group.goal->bottom_angle),
        ::std::abs(control_loops::claw_queue_group.status->separation -
                   control_loops::claw_queue_group.goal->separation_angle));
  }
  return ans;
}

bool ShooterIsReady() {
  control_loops::shooter_queue_group.goal.FetchLatest();
  control_loops::shooter_queue_group.status.FetchLatest();
  if (control_loops::shooter_queue_group.status->ready) {
    LOG(INFO, "Power error is %f - %f -> %f, ready %d\n",
        control_loops::shooter_queue_group.status->hard_stop_power,
        control_loops::shooter_queue_group.goal->shot_power,
        ::std::abs(control_loops::shooter_queue_group.status->hard_stop_power -
                   control_loops::shooter_queue_group.goal->shot_power),
        control_loops::shooter_queue_group.status->ready);
  }
  return (::std::abs(
              control_loops::shooter_queue_group.status->hard_stop_power -
              control_loops::shooter_queue_group.goal->shot_power) < 1.0) &&
         control_loops::shooter_queue_group.status->ready;
}

bool ShootAction::DoneSetupShot() {
  if (!control_loops::shooter_queue_group.status.FetchLatest()) {
    control_loops::shooter_queue_group.status.FetchNextBlocking();
  }
  if (!control_loops::claw_queue_group.status.FetchLatest()) {
    control_loops::claw_queue_group.status.FetchNextBlocking();
  }
  // Make sure that both the shooter and claw have reached the necessary
  // states.
  if (ShooterIsReady() && ClawIsReady()) {
    LOG(INFO, "Claw and Shooter ready for shooting.\n");
    return true;
  }

  return false;
}

bool ShootAction::DonePreShotOpen() {
  if (!control_loops::claw_queue_group.status.FetchLatest()) {
    control_loops::claw_queue_group.status.FetchNextBlocking();
  }
  if (control_loops::claw_queue_group.status->separation >
      kClawShootingSeparation) {
    LOG(INFO, "Opened up enough to shoot.\n");
    return true;
  }
  return false;
}

bool ShootAction::DoneShot() {
  if (!control_loops::shooter_queue_group.status.FetchLatest()) {
    control_loops::shooter_queue_group.status.FetchNextBlocking();
  }
  if (control_loops::shooter_queue_group.status->shots > previous_shots_) {
    LOG(INFO, "Shot succeeded!\n");
    return true;
  }
  return false;
}

::std::unique_ptr<TypedAction< ::frc971::actions::ShootActionQueueGroup>>
MakeShootAction() {
  return ::std::unique_ptr<
      TypedAction< ::frc971::actions::ShootActionQueueGroup>>(
      new TypedAction< ::frc971::actions::ShootActionQueueGroup>(
          &::frc971::actions::shoot_action));
}

}  // namespace actions
}  // namespace frc971

