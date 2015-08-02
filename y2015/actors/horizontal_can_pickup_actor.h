#ifndef Y2015_ACTORS_HORIZONTAL_CAN_PICKUP_ACTOR_H_
#define Y2015_ACTORS_HORIZONTAL_CAN_PICKUP_ACTOR_H_

#include <stdint.h>

#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "y2015/actors/horizontal_can_pickup_action.q.h"
#include "y2015/actors/fridge_profile_lib.h"

namespace frc971 {
namespace actors {

class HorizontalCanPickupActor
    : public FridgeActorBase<HorizontalCanPickupActionQueueGroup> {
 public:
  explicit HorizontalCanPickupActor(
      HorizontalCanPickupActionQueueGroup *queues);

  bool RunAction(const HorizontalCanPickupParams &params) override;

 private:
  // Waits until we are near the angle.
  // Returns false if we should cancel.
  bool WaitUntilNear(double angle);
  bool WaitUntilGoalNear(double angle);

  void MoveArm(double angle, double intake_power);
  void MoveArm(double angle, double intake_power,
               const ProfileParams profile_params);
};

typedef aos::common::actions::TypedAction<HorizontalCanPickupActionQueueGroup>
    HorizontalCanPickupAction;

// Makes a new HorizontalCanPickupActor action.
::std::unique_ptr<HorizontalCanPickupAction> MakeHorizontalCanPickupAction(
    const HorizontalCanPickupParams &params);

}  // namespace actors
}  // namespace frc971

#endif  // Y2015_ACTORS_HORIZONTAL_CAN_PICKUP_ACTOR_H_
