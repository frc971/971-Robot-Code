#include "aos/common/control_loop/Timing.h"
#include "aos/common/logging/logging.h"

#include "frc971/actions/shoot_action.h"
#include "frc971/control_loops/shooter/shooter.q.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/constants.h"

namespace frc971 {
namespace actions {

ShootAction::ShootAction(actions::ShootActionQueueGroup* s)
    : actions::ActionBase<actions::ShootActionQueueGroup>(s) {}

void ShootAction::RunAction() {
  if (!control_loops::shooter_queue_group.goal.MakeWithBuilder().shot_power(
          shoot_action.goal->shot_power)
          .shot_requested(false).unload_requested(false).load_requested(false)
          .Send()) {
    LOG(ERROR, "Failed to send the shoot action\n");
    return;
  }

  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          shoot_action.goal->shot_angle)
          .separation_angle(0.0).intake(2.0).centering(1.0).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // Make sure we have the latest statuses.
  control_loops::shooter_queue_group.status.FetchLatest();
  control_loops::claw_queue_group.status.FetchLatest();
  while (true) {
    // Make sure that both the shooter and claw have reached the necessary
    // states.
    if (control_loops::shooter_queue_group.status->ready &&
        control_loops::claw_queue_group.status->done) {
      LOG(INFO, "Claw and Shooter ready for shooting.\n");
      // TODO(james): Get realer numbers for shooter_action.
      break;
    }

    // Wait until we have a new status.
    control_loops::shooter_queue_group.status.FetchNextBlocking();
    control_loops::claw_queue_group.status.FetchNextBlocking();

    if (ShouldCancel()) return;
  }

  const frc971::constants::Values& values = frc971::constants::GetValues();

  // Open up the claw in preparation for shooting.
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          shoot_action.goal->shot_angle)
          .separation_angle(values.shooter_action.claw_separation_goal)
          .intake(2.0).centering(1.0).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  if (ShouldCancel()) return;

  // Make sure we have the latest status.
  control_loops::claw_queue_group.status.FetchLatest();
  while (true) {
    if (control_loops::claw_queue_group.status->separation >
        values.shooter_action.claw_shooting_separation) {
      LOG(INFO, "Opened up enough to shoot.\n");
      break;
    }

    // Wait until we have a new status.
    control_loops::claw_queue_group.status.FetchNextBlocking();

    if (ShouldCancel()) return;
  }

  // Shoot!
  if (!control_loops::shooter_queue_group.goal.MakeWithBuilder().shot_power(
          shoot_action.goal->shot_power)
          .shot_requested(true).unload_requested(false).load_requested(false)
          .Send()) {
    LOG(WARNING, "sending shooter goal failed\n");
    return;
  }

  if (ShouldCancel()) return;

  // Make sure that we have the latest shooter status.
  control_loops::shooter_queue_group.status.FetchLatest();
  // Get the number of shots fired up to this point. This should not be updated
  // again for another few cycles.
  int previous_shots = control_loops::shooter_queue_group.status->shots;
  while (true) {
    if (control_loops::shooter_queue_group.status->shots > previous_shots) {
      LOG(INFO, "Shot succeeded!\n");
      break;
    }

    // Wait until we have a new status.
    control_loops::shooter_queue_group.status.FetchNextBlocking();

    if (ShouldCancel()) return;
  }

  return;
}

}  // namespace actions
}  // namespace frc971
