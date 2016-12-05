#include <math.h>

#include "aos/common/time.h"
#include "y2015/actors/lift_actor.h"
#include "y2015/constants.h"
#include "y2015/actors/fridge_profile_lib.h"
#include "y2015/control_loops/claw/claw.q.h"

namespace y2015 {
namespace actors {
namespace {
constexpr ProfileParams kArmMove{0.6, 1.0};
constexpr ProfileParams kElevatorMove{0.9, 3.0};
constexpr ProfileParams kElevatorFixMove{0.9, 2.0};
}  // namespace

using ::y2015::control_loops::claw_queue;
using ::y2015::control_loops::fridge::fridge_queue;

LiftActor::LiftActor(LiftActionQueueGroup *queues)
    : FridgeActorBase<LiftActionQueueGroup>(queues) {}

bool LiftActor::RunAction(const LiftParams &params) {
  fridge_queue.status.FetchLatest();
  if (!fridge_queue.status.get()) {
    return false;
  }

  double goal_height = params.lift_height;
  double goal_angle = 0.0;

  if (params.second_lift) {
    DoFridgeProfile(params.intermediate_lift_height, 0.0, kElevatorFixMove,
                    kArmMove, fridge_queue.status->grabbers.top_front,
                    fridge_queue.status->grabbers.bottom_front,
                    fridge_queue.status->grabbers.bottom_back);
    if (ShouldCancel()) return true;
  }

  if (!StartFridgeProfile(params.lift_height, 0.0, kElevatorMove, kArmMove,
                          fridge_queue.status->grabbers.top_front,
                          fridge_queue.status->grabbers.bottom_front,
                          fridge_queue.status->grabbers.bottom_back)) {
    return true;
  }

  bool has_started_back = false;
  while (true) {
    if (fridge_queue.status->goal_height > 0.1) {
      if (!has_started_back) {
        if (!StartFridgeProfile(params.lift_height, params.lift_arm,
                                kElevatorMove, kArmMove,
                                fridge_queue.status->grabbers.top_front,
                                fridge_queue.status->grabbers.bottom_front,
                                fridge_queue.status->grabbers.bottom_back)) {
          return true;
        }
        goal_angle = params.lift_arm;
        has_started_back = true;
        if (params.pack_claw) {
          auto message = claw_queue.goal.MakeMessage();
          message->angle = params.pack_claw_angle;
          message->angular_velocity = 0.0;
          message->intake = 0.0;
          message->rollers_closed = true;
          message->max_velocity = 6.0;
          message->max_acceleration = 10.0;

          LOG_STRUCT(DEBUG, "Sending claw goal", *message);
          message.Send();
        }
      }
    }

    ProfileStatus status =
        IterateProfile(goal_height, goal_angle, kElevatorMove, kArmMove,
                       fridge_queue.status->grabbers.top_front,
                       fridge_queue.status->grabbers.bottom_front,
                       fridge_queue.status->grabbers.bottom_back);
    if (status == DONE || status == CANCELED) {
      return true;
    }
  }
  return true;
}

::std::unique_ptr<LiftAction> MakeLiftAction(const LiftParams &params) {
  return ::std::unique_ptr<LiftAction>(
      new LiftAction(&::y2015::actors::lift_action, params));
}

}  // namespace actors
}  // namespace y2015
