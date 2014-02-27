#include "stdio.h"

#include "aos/common/control_loop/Timing.h"
#include "aos/common/logging/logging.h"
#include "aos/common/network/team_number.h"

#include "frc971/actions/shoot_action.q.h"
#include "frc971/constants.h"

namespace frc971 {
namespace actions {

class ShootAction {
 public:
  void Run() {
    ::frc971::actions::shoot_action.FetchLatest();
    while (!::frc971::actions::shoot_action.get()) {
      ::frc971::actions::shoot_action.FetchNextBlocking();
    }

    while (true) {
      while (!::frc971::actions::shoot_action->run) {
        LOG(INFO, "Waiting for an action request.\n");
        ::frc971::actions::shoot_action.FetchNextBlocking();
      }
      LOG(INFO, "Starting action\n");
      RunAction();

      while (::frc971::actions::shoot_action->run) {
        LOG(INFO, "Waiting for the action to be stopped.\n");
        ::frc971::actions::shoot_action.FetchNextBlocking();
      }
    }
  }

  void RunAction();

 protected:
  // Returns true if the action should be canceled.
  bool ShouldCancel() {
    shoot_action.FetchLatest();
    bool ans = !::frc971::actions::shoot_action->run;
    if (ans) {
      LOG(INFO, "Time to exit auto mode\n");
    }
    return ans;
  }
};

void ShootAction::RunAction() {
  if (!control_loops::shooter_queue_group.goal.MakeWithBuilder().shot_power(
          shoot_action->shot_power).shot_requested(false)
          .unload_requested(false).load_requested(false).Send()) {
    LOG(ERROR, "Failed to send the shoot action\n");
    return;
  }

  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          shoot_action->shot_angle).separation_angle(0.0).intake(2.0)
          .centering(1.0).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  // Make sure we have the latest statuses.
  control_loops::shooter_queue_group.status.FetchLatest();
  control_loops::claw_queue_group.status.FetchLatest();
  while (true) {
    if (control_loops::shooter_queue_group.status->ready &&
        control_loops::claw_queue_group.status->done) {
      LOG(INFO, "Claw and Shooter ready for shooting.\n");
      break;
    }

    // Wait until we have a new status.
    control_loops::shooter_queue_group.status.FetchNextBlocking();
    control_loops::claw_queue_group.status.FetchNextBlocking();

    if (ShouldCancel()) return;
  }

  // Open up the claw in preparation for shooting.
  if (!control_loops::claw_queue_group.goal.MakeWithBuilder().bottom_angle(
          shoot_action->shot_angle).separation_angle(k2).intake(2.0)
          .centering(1.0).Send()) {
    LOG(WARNING, "sending claw goal failed\n");
    return;
  }

  if (ShouldCancel()) return;

  // Make sure we have the latest statuses.
  control_loops::claw_queue_group.status.FetchLatest();
  while (true) {
    if (control_loops::claw_queue_group.status->separation > k1) {
      LOG(INFO, "Opened up enough to shoot.\n");
      break;
    }

    // Wait until we have a new status.
    control_loops::claw_queue_group.status.FetchNextBlocking();

    if (ShouldCancel()) return;
  }
}

}  // namespace actions
}  // namespace frc971
