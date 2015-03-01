#ifndef FRC971_ACTORS_CAN_PICKUP_ACTOR_H_
#define FRC971_ACTORS_CAN_PICKUP_ACTOR_H_

#include <stdint.h>

#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "frc971/actors/can_pickup_action.q.h"

namespace frc971 {
namespace actors {

class CanPickupActor
    : public aos::common::actions::ActorBase<CanPickupActionQueueGroup> {
 public:
  explicit CanPickupActor(CanPickupActionQueueGroup *queues);

  void DoProfile(double height, double angle, bool grabbers);

  bool RunAction(const CanPickupParams &params) override;
};

typedef aos::common::actions::TypedAction<CanPickupActionQueueGroup>
    CanPickupAction;

// Makes a new CanPickupActor action.
::std::unique_ptr<CanPickupAction> MakeCanPickupAction(
    const CanPickupParams &params);

}  // namespace actors
}  // namespace frc971

#endif
