#include "y2015/actors/stack_and_lift_actor.h"

#include <math.h>

#include "aos/common/controls/control_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"

#include "y2015/constants.h"
#include "y2015/control_loops/claw/claw.q.h"
#include "y2015/actors/stack_actor.h"
#include "y2015/actors/lift_actor.h"

namespace y2015 {
namespace actors {

namespace chrono = ::std::chrono;
using ::y2015::control_loops::claw_queue;
using ::y2015::control_loops::fridge::fridge_queue;

StackAndLiftActor::StackAndLiftActor(StackAndLiftActionQueueGroup *queues)
    : FridgeActorBase<StackAndLiftActionQueueGroup>(queues) {}

bool StackAndLiftActor::RunAction(const StackAndLiftParams &params) {
  claw_queue.goal.FetchLatest();
  double claw_goal_start;
  bool have_claw_goal_start;
  if (claw_queue.goal.get()) {
    claw_goal_start = claw_queue.goal->angle;
    have_claw_goal_start = true;
  } else {
    claw_goal_start = 0;
    have_claw_goal_start = false;
  }

  {
    StackParams stack_params = params.stack_params;
    stack_params.only_place = false;
    ::std::unique_ptr<StackAction> stack_action =
        MakeStackAction(stack_params);
    stack_action->Start();
    ::aos::time::PhasedLoop phased_loop(::aos::controls::kLoopFrequency,
                                        ::std::chrono::milliseconds(5) / 2);
    while (stack_action->Running()) {
      phased_loop.SleepUntilNext();

      if (ShouldCancel()) {
        stack_action->Cancel();
        LOG(WARNING, "Cancelling fridge and claw.\n");
        return true;
      }
    }
  }

  {
    fridge_queue.goal.FetchLatest();
    if (!fridge_queue.goal.get()) {
      return false;
    }
    auto new_fridge_goal = fridge_queue.goal.MakeMessage();
    *new_fridge_goal = *fridge_queue.goal;
    new_fridge_goal->grabbers.top_front = params.grab_after_stack;
    new_fridge_goal->grabbers.top_back = params.grab_after_stack;
    new_fridge_goal->grabbers.bottom_front = params.grab_after_stack;
    new_fridge_goal->grabbers.bottom_back = params.grab_after_stack;
    if (!new_fridge_goal.Send()) {
      LOG(ERROR, "Failed to send fridge close.\n");
      return false;
    }
  }

  if (!WaitOrCancel(chrono::duration_cast<::aos::monotonic_clock::duration>(
          chrono::duration<double>(params.clamp_pause_time)))) {
    return true;
  }

  {
    auto lift_params = params.lift_params;
    lift_params.pack_claw = have_claw_goal_start;
    lift_params.pack_claw_angle = claw_goal_start;
    ::std::unique_ptr<LiftAction> lift_action = MakeLiftAction(lift_params);
    lift_action->Start();
    ::aos::time::PhasedLoop phased_loop(::aos::controls::kLoopFrequency,
                                        ::std::chrono::milliseconds(5) / 2);
    while (lift_action->Running()) {
      phased_loop.SleepUntilNext();

      if (ShouldCancel()) {
        lift_action->Cancel();
        LOG(WARNING, "Cancelling fridge and claw.\n");
        return true;
      }
    }
  }

  {
    fridge_queue.goal.FetchLatest();
    if (!fridge_queue.goal.get()) {
      return false;
    }
    auto new_fridge_goal = fridge_queue.goal.MakeMessage();
    *new_fridge_goal = *fridge_queue.goal;
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
      new StackAndLiftAction(&::y2015::actors::stack_and_lift_action, params));
}

}  // namespace actors
}  // namespace y2015
