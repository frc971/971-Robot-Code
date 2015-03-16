#include <math.h>

#include "aos/common/time.h"
#include "frc971/actors/lift_actor.h"
#include "frc971/constants.h"
#include "frc971/actors/fridge_profile_lib.h"

namespace frc971 {
namespace actors {
namespace {
constexpr ProfileParams kArmMove{0.6, 1.0};
constexpr ProfileParams kElevatorMove{0.9, 3.0};
}  // namespace

LiftActor::LiftActor(LiftActionQueueGroup *queues)
    : FridgeActorBase<LiftActionQueueGroup>(queues) {}

bool LiftActor::RunAction(const LiftParams &params) {
  control_loops::fridge_queue.status.FetchLatest();
  if (!control_loops::fridge_queue.status.get()) {
    return false;
  }

  double goal_height = params.lift_height;
  double goal_angle = 0.0;

  if (!StartFridgeProfile(
          params.lift_height, 0.0, kElevatorMove, kArmMove,
          control_loops::fridge_queue.status->grabbers.top_front,
          control_loops::fridge_queue.status->grabbers.bottom_front,
          control_loops::fridge_queue.status->grabbers.bottom_back)) {
    return true;
  }

  bool has_started_back = false;
  while (true) {
    if (control_loops::fridge_queue.status->goal_height > 0.1) {
      if (!has_started_back) {
        if (!StartFridgeProfile(
                params.lift_height, params.lift_arm, kElevatorMove, kArmMove,
                control_loops::fridge_queue.status->grabbers.top_front,
                control_loops::fridge_queue.status->grabbers.bottom_front,
                control_loops::fridge_queue.status->grabbers.bottom_back)) {
          return true;
        }
        goal_angle = params.lift_arm;
        has_started_back = true;
      }
    }

    ProfileStatus status = IterateProfile(
        goal_height, goal_angle, kElevatorMove, kArmMove,
        control_loops::fridge_queue.status->grabbers.top_front,
        control_loops::fridge_queue.status->grabbers.bottom_front,
        control_loops::fridge_queue.status->grabbers.bottom_back);
    if (status == DONE || status == CANCELED) {
      return true;
    }
  }
  return true;
}

::std::unique_ptr<LiftAction> MakeLiftAction(const LiftParams &params) {
  return ::std::unique_ptr<LiftAction>(
      new LiftAction(&::frc971::actors::lift_action, params));
}

}  // namespace actors
}  // namespace frc971
