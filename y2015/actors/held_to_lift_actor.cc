#include "y2015/actors/held_to_lift_actor.h"

#include <math.h>

#include "aos/common/time.h"
#include "y2015/constants.h"
#include "y2015/actors/fridge_profile_lib.h"
#include "y2015/actors/lift_actor.h"
#include "y2015/control_loops/claw/claw.q.h"

namespace frc971 {
namespace actors {
namespace {
constexpr ProfileParams kArmMove{0.6, 2.0};
constexpr ProfileParams kFastArmMove{1.2, 4.0};
constexpr ProfileParams kElevatorMove{0.9, 3.0};
constexpr ProfileParams kFastElevatorMove{1.2, 5.0};
}  // namespace

HeldToLiftActor::HeldToLiftActor(HeldToLiftActionQueueGroup *queues)
    : FridgeActorBase<HeldToLiftActionQueueGroup>(queues) {}

bool HeldToLiftActor::RunAction(const HeldToLiftParams &params) {
  control_loops::fridge_queue.status.FetchLatest();
  if (!control_loops::fridge_queue.status.get()) {
    return false;
  }

  // Move claw out of the way.
  {
    bool send_goal = true;
    double claw_goal = params.claw_out_angle;
    control_loops::claw_queue.status.FetchLatest();
    if (control_loops::claw_queue.status.get()) {
      if (control_loops::claw_queue.status->goal_angle < claw_goal) {
        send_goal = false;
      }
    }
    if (send_goal) {
      auto message = control_loops::claw_queue.goal.MakeMessage();
      message->angle = params.claw_out_angle;
      message->angular_velocity = 0.0;
      message->intake = 0.0;
      message->max_velocity = 6.0;
      message->max_acceleration = 10.0;
      message->rollers_closed = true;

      LOG_STRUCT(DEBUG, "Sending claw goal", *message);
      message.Send();
    }
  }

  control_loops::fridge_queue.status.FetchLatest();
  if (!control_loops::fridge_queue.status.get()) {
    return false;
  }

  if (control_loops::fridge_queue.status->goal_height != params.bottom_height ||
      control_loops::fridge_queue.status->goal_angle != 0.0) {
    // Lower with the fridge clamps open and move it forwards slightly to clear.
    DoFridgeProfile(control_loops::fridge_queue.status->goal_height,
                    params.arm_clearance, kFastElevatorMove, kFastArmMove,
                    false);
    if (ShouldCancel()) return true;

    DoFridgeProfile(params.bottom_height, params.arm_clearance,
                    kFastElevatorMove, kFastArmMove, false);
    if (ShouldCancel()) return true;

    // Move it back to the storage location.
    DoFridgeProfile(params.bottom_height, 0.0, kElevatorMove, kArmMove, false);
    if (ShouldCancel()) return true;

    if (!WaitOrCancel(
            aos::time::Time::InSeconds(params.before_lift_settle_time))) {
      return true;
    }

    // Clamp
    DoFridgeProfile(params.bottom_height, 0.0, kElevatorMove, kArmMove, true);
    if (ShouldCancel()) return true;

    if (!WaitOrCancel(aos::time::Time::InSeconds(params.clamp_pause_time))) {
      return true;
    }
  }

  {
    ::std::unique_ptr<LiftAction> lift_action =
        MakeLiftAction(params.lift_params);
    lift_action->Start();
    while (lift_action->Running()) {
      ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(),
                                 2500);

      if (ShouldCancel()) {
        lift_action->Cancel();
        LOG(WARNING, "Cancelling fridge and claw.\n");
        return true;
      }
    }
  }

  return true;
}

::std::unique_ptr<HeldToLiftAction> MakeHeldToLiftAction(
    const HeldToLiftParams &params) {
  return ::std::unique_ptr<HeldToLiftAction>(
      new HeldToLiftAction(&::frc971::actors::held_to_lift_action, params));
}

}  // namespace actors
}  // namespace frc971
