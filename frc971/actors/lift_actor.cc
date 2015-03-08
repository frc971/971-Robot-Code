#include <math.h>

#include "aos/common/time.h"
#include "frc971/actors/lift_actor.h"
#include "frc971/constants.h"
#include "frc971/actors/fridge_profile_lib.h"

namespace frc971 {
namespace actors {
namespace {
constexpr ProfileParams kArmMove{0.6, 2.0};
constexpr ProfileParams kElevatorMove{0.9, 3.0};
}  // namespace

LiftActor::LiftActor(LiftActionQueueGroup *queues)
    : FridgeActorBase<LiftActionQueueGroup>(queues) {}

bool LiftActor::RunAction(const LiftParams &params) {
  // Lift the box straight up.
  DoFridgeProfile(params.lift_height, 0.0, kElevatorMove, kArmMove, true);
  // Move it back to the storage location.
  DoFridgeProfile(params.lift_height, params.lift_arm, kElevatorMove, kArmMove,
                  true);

  return true;
}

::std::unique_ptr<LiftAction> MakeLiftAction(const LiftParams &params) {
  return ::std::unique_ptr<LiftAction>(
      new LiftAction(&::frc971::actors::lift_action, params));
}

}  // namespace actors
}  // namespace frc971
