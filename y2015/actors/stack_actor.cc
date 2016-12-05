#include "y2015/actors/stack_actor.h"

#include <math.h>

#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"

#include "y2015/constants.h"
#include "y2015/control_loops/claw/claw.q.h"

namespace y2015 {
namespace actors {
namespace {
constexpr ProfileParams kArmWithStackMove{1.75, 4.20};
constexpr ProfileParams kSlowArmMove{1.3, 1.4};
constexpr ProfileParams kSlowElevatorMove{1.0, 3.0};

constexpr ProfileParams kFastArmMove{0.8, 4.0};
constexpr ProfileParams kFastElevatorMove{1.2, 5.0};
constexpr ProfileParams kReallyFastElevatorMove{1.2, 6.0};
}  // namespace

using ::y2015::control_loops::claw_queue;
using ::y2015::control_loops::fridge::fridge_queue;

StackActor::StackActor(StackActionQueueGroup *queues)
    : FridgeActorBase<StackActionQueueGroup>(queues) {}

bool StackActor::RunAction(const StackParams &params) {
  const auto &values = constants::GetValues();

  fridge_queue.status.FetchLatest();
  if (!fridge_queue.status.get()) {
    LOG(ERROR, "Got no fridge status packet.\n");
    return false;
  }

  // If we are really high, probably have a can.  Move over before down.
  if (fridge_queue.status->goal_height >
      params.over_box_before_place_height + 0.1) {
    // Set the current stack down on top of the bottom box.
    DoFridgeProfile(fridge_queue.status->goal_height, 0.0,
                    kSlowElevatorMove, kArmWithStackMove, true);
    if (ShouldCancel()) return true;
  }

  // Set the current stack down on top of the bottom box.
  DoFridgeProfile(params.bottom + values.tote_height, 0.0, kSlowElevatorMove,
                  kSlowArmMove, true);
  if (ShouldCancel()) return true;

  // Clamp.
  if (!params.only_place) {
    // Move the claw out of the way only if we are supposed to pick up.
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

  if (params.only_place) {
    // open grabber for place only
    DoFridgeProfile(params.bottom + values.tote_height, 0.0, kFastElevatorMove,
                    kFastArmMove, false);
    // Finish early if we aren't supposed to grab.
    return true;
  }

  if (ShouldCancel()) return true;
  // grab can (if in fang mode the grabber stays closed)
  DoFridgeProfile(params.bottom, 0.0, kReallyFastElevatorMove, kFastArmMove, true,
                  true, true);

  return true;
}

::std::unique_ptr<StackAction> MakeStackAction(const StackParams &params) {
  return ::std::unique_ptr<StackAction>(
      new StackAction(&::y2015::actors::stack_action, params));
}

}  // namespace actors
}  // namespace y2015
