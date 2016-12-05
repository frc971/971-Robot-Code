#include <math.h>

#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"

#include "y2015/actors/can_pickup_actor.h"
#include "y2015/constants.h"
#include "y2015/control_loops/claw/claw.q.h"

using ::y2015::control_loops::fridge::fridge_queue;
using ::y2015::control_loops::claw_queue;

namespace y2015 {
namespace actors {
namespace {
constexpr ProfileParams kHorizontalMove{1.1, 1.8};
constexpr ProfileParams kVerticalMove{0.3, 2.0};
constexpr ProfileParams kFastHorizontalMove{1.25, 5.0};
constexpr ProfileParams kFastVerticalMove{0.40, 2.0};
constexpr ProfileParams kPureVerticalMove{1.20, 5.0};
}  // namespace

CanPickupActor::CanPickupActor(CanPickupActionQueueGroup *queues)
    : FridgeActorBase<CanPickupActionQueueGroup>(queues) {}

double CanPickupActor::CurrentGoalX() {
  fridge_queue.status.FetchLatest();
  if (!fridge_queue.status.get()) {
    LOG(ERROR, "Reading from fridge status queue failed.\n");
    return 0.0;
  }

  return fridge_queue.status->goal_x;
}

double CanPickupActor::CurrentGoalHeight() {
  fridge_queue.status.FetchLatest();
  if (!fridge_queue.status.get()) {
    LOG(ERROR, "Reading from fridge status queue failed.\n");
    return 0.0;
  }

  return fridge_queue.status->goal_y;
}

bool CanPickupActor::RunAction(const CanPickupParams &params) {
  // Make sure the claw is down.
  {
    auto message = claw_queue.goal.MakeMessage();
    message->angle = 0.0;
    message->angular_velocity = 0.0;
    message->intake = 0.0;
    message->rollers_closed = true;
    message->max_velocity = 6.0;
    message->max_acceleration = 10.0;

    LOG_STRUCT(DEBUG, "Sending claw down goal", *message);
    message.Send();
  }

  // Go around the can.
  DoFridgeXYProfile(params.pickup_x, params.pickup_y, kFastHorizontalMove,
                    kFastVerticalMove, true, false, false);
  if (ShouldCancel()) return true;

  if (!StartFridgeXYProfile(
          params.pickup_x, params.pickup_goal_before_move_height,
          kHorizontalMove, kPureVerticalMove, true, true, true)) {
    return false;
  }
  {
    auto message = claw_queue.goal.MakeMessage();
    message->angle = 0.0;
    message->angular_velocity = 0.0;
    message->intake = 0.0;
    message->rollers_closed = false;
    message->max_velocity = 6.0;
    message->max_acceleration = 10.0;

    LOG_STRUCT(DEBUG, "Sending claw down goal", *message);
    message.Send();
  }

  bool above_claw = false;
  while (true) {
    if (CurrentGoalHeight() > params.lift_height && !above_claw) {
      if (!StartFridgeXYProfile(0.0, params.before_place_height,
                                kHorizontalMove, kVerticalMove, true, true,
                                true)) {
        return false;
      }
      above_claw = true;
    }
    if (CurrentGoalX() < params.start_lowering_x) {
      // Getting close, start lowering.
      LOG(DEBUG, "Starting to lower the can onto the tray.\n");
      break;
    }
    ProfileStatus status =
        IterateXYProfile(0.0, params.before_place_height, kHorizontalMove,
                         kVerticalMove, true, true, true);
    if (status == DONE || status == CANCELED) {
      break;
    }
  }
  if (ShouldCancel()) return true;

  // Lower it.
  DoFridgeXYProfile(0.0, params.end_height, kHorizontalMove, kPureVerticalMove,
                    true);
  if (ShouldCancel()) return true;

  return true;
}

::std::unique_ptr<CanPickupAction> MakeCanPickupAction(
    const CanPickupParams &params) {
  return ::std::unique_ptr<CanPickupAction>(
      new CanPickupAction(&::y2015::actors::can_pickup_action, params));
}

}  // namespace actors
}  // namespace y2015
