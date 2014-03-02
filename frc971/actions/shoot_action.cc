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
  const frc971::constants::Values& values = frc971::constants::GetValues();

  // Set shot power
  if (!control_loops::shooter_queue_group.goal.MakeWithBuilder().shot_power(
          shoot_action.goal->shot_power)
          .shot_requested(false).unload_requested(false).load_requested(false)
          .Send()) {
    LOG(ERROR, "Failed to send the shoot action\n");
    return;
  }

  // Set claw angle
  control_loops::drivetrain.status.FetchLatest();
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          shoot_action.goal->shot_angle +
          SpeedToAngleOffset(control_loops::drivetrain.status->robot_speed))
          .separation_angle(0.0).intake(2.0).centering(1.0).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // wait for claw to be ready
  if (WaitUntil(::std::bind(&ShootAction::DoneSetupShot, this))) return;

  // Open up the claw in preparation for shooting.
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          shoot_action.goal->shot_angle)
          .separation_angle(values.shooter_action.claw_separation_goal)
          .intake(2.0).centering(1.0).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // wait for the claw to open up a little before we shoot
  if (WaitUntil(::std::bind(&ShootAction::DonePreShotOpen, this))) return;

  // Make sure that we have the latest shooter status.
  control_loops::shooter_queue_group.status.FetchLatest();
  // Get the number of shots fired up to this point. This should not be updated
  // again for another few cycles.
  previous_shots_ = control_loops::shooter_queue_group.status->shots;
  // Shoot!
  if (!control_loops::shooter_queue_group.goal.MakeWithBuilder().shot_power(
          shoot_action.goal->shot_power)
          .shot_requested(true).unload_requested(false).load_requested(false)
          .Send()) {
    LOG(WARNING, "sending shooter goal failed\n");
    return;
  }

  // wait for record of shot having been fired
  if (WaitUntil(::std::bind(&ShootAction::DoneShot, this))) return;
  
  // done with action
  return;
}

bool ShootAction::DoneSetupShot() {
  control_loops::shooter_queue_group.status.FetchNextBlocking();
  control_loops::claw_queue_group.status.FetchNextBlocking();
  // Make sure that both the shooter and claw have reached the necessary
  // states.
  if (control_loops::shooter_queue_group.status->ready &&
      control_loops::claw_queue_group.status->done) {
    LOG(INFO, "Claw and Shooter ready for shooting.\n");
    // TODO(james): Get realer numbers for shooter_action.
    return true;
  }

  // update the claw position to track velocity
  // TODO(ben): the claw may never reach the goal if the velocity is
  // continually changing, we will need testing to see
  control_loops::drivetrain.status.FetchLatest();
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          shoot_action.goal->shot_angle +
          SpeedToAngleOffset(control_loops::drivetrain.status->robot_speed))
          .separation_angle(0.0).intake(2.0).centering(1.0).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    abort_ = true;
    return true;
  } else {
    LOG(INFO, "Updating claw angle for velocity offset(%.4f).\n",
        SpeedToAngleOffset(control_loops::drivetrain.status->robot_speed));
  }
  return false;
}

bool ShootAction::DonePreShotOpen() {
  const frc971::constants::Values& values = frc971::constants::GetValues();
  control_loops::claw_queue_group.status.FetchNextBlocking();
  if (control_loops::claw_queue_group.status->separation >
      values.shooter_action.claw_shooting_separation) {
    LOG(INFO, "Opened up enough to shoot.\n");
    return true;
  }
  return false;
}

bool ShootAction::DoneShot() {
  control_loops::shooter_queue_group.status.FetchNextBlocking();
  if (control_loops::shooter_queue_group.status->shots > previous_shots_) {
    LOG(INFO, "Shot succeeded!\n");
    return true;
  }
  return false;
}

}  // namespace actions
}  // namespace frc971
