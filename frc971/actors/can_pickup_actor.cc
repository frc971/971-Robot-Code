#include <math.h>

#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"

#include "frc971/actors/can_pickup_actor.h"
#include "frc971/constants.h"
#include "frc971/control_loops/claw/claw.q.h"

namespace frc971 {
namespace actors {
namespace {
constexpr ProfileParams kArmMove{1.0, 1.6};
constexpr ProfileParams kElevatorMove{0.6, 2.2};
}  // namespace

CanPickupActor::CanPickupActor(CanPickupActionQueueGroup *queues)
    : FridgeActorBase<CanPickupActionQueueGroup>(queues) {}

bool CanPickupActor::RunAction(const CanPickupParams &params) {
  // Go around the can.
  DoFridgeProfile(params.pickup_height, params.pickup_angle, kElevatorMove,
                  kArmMove, false);
  if (ShouldCancel()) return true;

  // Lift and grab.
  DoFridgeProfile(params.lift_height, params.pickup_angle, kElevatorMove,
                  kArmMove, true);
  if (ShouldCancel()) return true;

  // Pull it back in.
  DoFridgeProfile(params.lift_height, params.end_angle, kElevatorMove, kArmMove,
                  true);
  if (ShouldCancel()) return true;

  // Pull it back in.
  DoFridgeProfile(params.end_height, params.end_angle, kElevatorMove, kArmMove,
                  true);
  if (ShouldCancel()) return true;

  return true;
}

::std::unique_ptr<CanPickupAction> MakeCanPickupAction(
    const CanPickupParams &params) {
  return ::std::unique_ptr<CanPickupAction>(
      new CanPickupAction(&::frc971::actors::can_pickup_action, params));
}

}  // namespace actors
}  // namespace frc971
