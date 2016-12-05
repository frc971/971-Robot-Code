#include "y2015/actors/stack_and_hold_actor.h"

#include <math.h>

#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"

#include "y2015/constants.h"
#include "y2015/control_loops/claw/claw.q.h"
#include "y2015/actors/stack_actor.h"

namespace y2015 {
namespace actors {
namespace {
constexpr ProfileParams kReallySlowArmMove{0.1, 1.0};
constexpr ProfileParams kReallySlowElevatorMove{0.10, 1.0};

constexpr ProfileParams kFastArmMove{0.8, 4.0};
constexpr ProfileParams kFastElevatorMove{1.2, 4.0};
}  // namespace

namespace chrono = ::std::chrono;

using ::y2015::control_loops::claw_queue;

StackAndHoldActor::StackAndHoldActor(StackAndHoldActionQueueGroup *queues)
    : FridgeActorBase<StackAndHoldActionQueueGroup>(queues) {}

bool StackAndHoldActor::RunAction(const StackAndHoldParams &params) {
  // TODO(ben)): this action is no longer used (source Cameron) and my be broken
  // by the stack action having the grabbers closed at the end for the fangs. So
  // here I am disabling it until further information is provided.
  if (params.place_not_stack) {
    // Move the arm out of the way.
    {
      bool send_goal = true;
      claw_queue.status.FetchLatest();
      if (claw_queue.status.get()) {
        if (claw_queue.status->goal_angle <
            params.claw_out_angle) {
          send_goal = false;
        }
      }
      if (send_goal) {
        auto message = claw_queue.goal.MakeMessage();
        message->angle = params.claw_out_angle;
        message->angular_velocity = 0.0;
        message->intake = 0.0;
        message->rollers_closed = true;
        message->max_velocity = 6.0;
        message->max_acceleration = 10.0;

        LOG_STRUCT(DEBUG, "Sending claw goal", *message);
        message.Send();
      }
    }

    // Get close, but keep the arm forwards
    DoFridgeProfile(params.bottom + 0.04, -0.05, kFastArmMove,
                    kFastElevatorMove, true);

    // Lower and pull back.
    if (ShouldCancel()) return true;
    DoFridgeProfile(params.bottom, 0.0, kReallySlowArmMove,
                    kReallySlowElevatorMove, true, true, false);

    // Release.
    if (ShouldCancel()) return true;
    DoFridgeProfile(params.bottom, 0.0, kReallySlowArmMove,
                    kReallySlowElevatorMove, false);
  } else {
    StackParams stack_params;
    stack_params.only_place = true;
    stack_params.arm_clearance = params.arm_clearance;
    stack_params.claw_out_angle = params.claw_out_angle;
    stack_params.over_box_before_place_height =
        params.over_box_before_place_height;
    stack_params.bottom = params.bottom;
    ::std::unique_ptr<StackAction> stack_action =
        MakeStackAction(stack_params);
    stack_action->Start();
    while (stack_action->Running()) {
      ::aos::time::PhasedLoopXMS(chrono::duration_cast<chrono::milliseconds>(
                                     ::aos::controls::kLoopFrequency).count(),
                                 2500);

      if (ShouldCancel()) {
        stack_action->Cancel();
        LOG(WARNING, "Cancelling fridge and claw.\n");
        return true;
      }
    }
  }

  if (!WaitOrCancel(chrono::duration_cast<::aos::monotonic_clock::duration>(
          chrono::duration<double>(params.clamp_pause_time)))) {
    return true;
  }

  // Go up.
  DoFridgeProfile(params.hold_height, params.arm_clearance, kFastArmMove,
                  kFastElevatorMove, false);

  if (ShouldCancel()) return true;

  if (params.place_not_stack) {
    // Clamp the stack with the claw.
    auto message = claw_queue.goal.MakeMessage();
    message->angle = params.claw_clamp_angle;
    message->angular_velocity = 0.0;
    message->intake = 0.0;
    message->rollers_closed = true;
    message->max_velocity = 6.0;
    message->max_acceleration = 6.0;

    LOG_STRUCT(DEBUG, "Sending claw goal", *message);
    message.Send();
  }
  // Move back
  DoFridgeProfile(params.hold_height, 0.0, kFastArmMove, kFastElevatorMove,
                  false);
  if (ShouldCancel()) return true;
  // Grab
  DoFridgeProfile(params.hold_height, 0.0, kFastArmMove, kFastElevatorMove,
                  true);
  if (ShouldCancel()) return true;

  return true;
}

::std::unique_ptr<StackAndHoldAction> MakeStackAndHoldAction(
    const StackAndHoldParams &params) {
  return ::std::unique_ptr<StackAndHoldAction>(
      new StackAndHoldAction(&::y2015::actors::stack_and_hold_action, params));
}

}  // namespace actors
}  // namespace y2015
