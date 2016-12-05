#ifndef Y2015_ACTORS_CAN_PICKUP_ACTOR_H_
#define Y2015_ACTORS_CAN_PICKUP_ACTOR_H_

#include <stdint.h>

#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "y2015/actors/can_pickup_action.q.h"
#include "y2015/actors/fridge_profile_lib.h"

namespace y2015 {
namespace actors {

class CanPickupActor : public FridgeActorBase<CanPickupActionQueueGroup> {
 public:
  explicit CanPickupActor(CanPickupActionQueueGroup *queues);

  bool RunAction(const CanPickupParams &params) override;

 private:
  double CurrentGoalHeight();
  double CurrentGoalX();
};

typedef aos::common::actions::TypedAction<CanPickupActionQueueGroup>
    CanPickupAction;

// Makes a new CanPickupActor action.
::std::unique_ptr<CanPickupAction> MakeCanPickupAction(
    const CanPickupParams &params);

}  // namespace actors
}  // namespace y2015

#endif
