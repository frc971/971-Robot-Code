#include "frc971/actors/stack_and_lift_actor.h"

#include <math.h>

#include "aos/common/controls/control_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"

#include "frc971/constants.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/actors/stack_actor.h"
#include "frc971/actors/lift_actor.h"

namespace frc971 {
namespace actors {

StackAndLiftActor::StackAndLiftActor(StackAndLiftActionQueueGroup *queues)
    : FridgeActorBase<StackAndLiftActionQueueGroup>(queues) {}

bool StackAndLiftActor::RunAction(const StackAndLiftParams &params) {
  {
    StackParams stack_params = params.stack_params;
    stack_params.only_place = false;
    ::std::unique_ptr<StackAction> stack_action =
        MakeStackAction(stack_params);
    stack_action->Start();
    while (stack_action->Running()) {
      ::aos::time::PhasedLoopXMS(::aos::controls::kLoopFrequency.ToMSec(),
                                 2500);

      if (ShouldCancel()) {
        stack_action->Cancel();
        LOG(WARNING, "Cancelling fridge and claw.\n");
        return true;
      }
    }
  }

  {
    control_loops::fridge_queue.goal.FetchLatest();
    if (!control_loops::fridge_queue.goal.get()) {
      return false;
    }
    auto new_fridge_goal = control_loops::fridge_queue.goal.MakeMessage();
    *new_fridge_goal = *control_loops::fridge_queue.goal;
    new_fridge_goal->grabbers.top_front = params.grab_after_stack;
    new_fridge_goal->grabbers.top_back = params.grab_after_stack;
    new_fridge_goal->grabbers.bottom_front = params.grab_after_stack;
    new_fridge_goal->grabbers.bottom_back = params.grab_after_stack;
    if (!new_fridge_goal.Send()) {
      LOG(ERROR, "Failed to send fridge close.\n");
      return false;
    }
  }

  if (!WaitOrCancel(aos::time::Time::InSeconds(params.clamp_pause_time))) {
    return true;
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

  {
    control_loops::fridge_queue.goal.FetchLatest();
    if (!control_loops::fridge_queue.goal.get()) {
      return false;
    }
    auto new_fridge_goal = control_loops::fridge_queue.goal.MakeMessage();
    *new_fridge_goal = *control_loops::fridge_queue.goal;
    new_fridge_goal->grabbers.top_front = params.grab_after_lift;
    new_fridge_goal->grabbers.top_back = params.grab_after_lift;
    new_fridge_goal->grabbers.bottom_front = params.grab_after_lift;
    new_fridge_goal->grabbers.bottom_back = params.grab_after_lift;
    if (!new_fridge_goal.Send()) {
      LOG(ERROR, "Failed to send fridge close.\n");
      return false;
    }
  }
  return true;
}

::std::unique_ptr<StackAndLiftAction> MakeStackAndLiftAction(
    const StackAndLiftParams &params) {
  return ::std::unique_ptr<StackAndLiftAction>(
      new StackAndLiftAction(&::frc971::actors::stack_and_lift_action, params));
}

}  // namespace actors
}  // namespace frc971
