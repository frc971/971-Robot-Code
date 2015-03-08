#ifndef FRC971_ACTORS_HORIZONTAL_CAN_PICKUP_ACTOR_H_
#define FRC971_ACTORS_HORIZONTAL_CAN_PICKUP_ACTOR_H_

#include <stdint.h>

#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "frc971/actors/horizontal_can_pickup_action.q.h"
#include "frc971/actors/fridge_profile_lib.h"

namespace frc971 {
namespace actors {

class HorizontalCanPickupActor
    : public FridgeActorBase<HorizontalCanPickupActionQueueGroup> {
 public:
  explicit HorizontalCanPickupActor(
      HorizontalCanPickupActionQueueGroup *queues);

  bool RunAction(const HorizontalCanPickupParams &params) override;

 private:
  // Waits until the duration has elapsed, or we are asked to cancel.
  // Returns false if we should cancel.
  bool WaitOrCancel(::aos::time::Time duration);

  // Waits until we are near the angle.
  // Returns false if we should cancel.
  bool WaitUntilNear(double angle);

  void MoveArm(double angle, double intake_power);
};

typedef aos::common::actions::TypedAction<HorizontalCanPickupActionQueueGroup>
    HorizontalCanPickupAction;

// Makes a new HorizontalCanPickupActor action.
::std::unique_ptr<HorizontalCanPickupAction> MakeHorizontalCanPickupAction(
    const HorizontalCanPickupParams &params);

}  // namespace actors
}  // namespace frc971

#endif  // FRC971_ACTORS_HORIZONTAL_CAN_PICKUP_ACTOR_H_
