#ifndef FRC971_ACTORS_PICKUP_ACTOR_H_
#define FRC971_ACTORS_PICKUP_ACTOR_H_

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "frc971/actors/pickup_action.q.h"

namespace frc971 {
namespace actors {

class PickupActor
    : public aos::common::actions::ActorBase<PickupActionQueueGroup> {
 public:
  explicit PickupActor(PickupActionQueueGroup *queues);

  bool RunAction(const PickupParams &params) override;
};

typedef aos::common::actions::TypedAction<PickupActionQueueGroup> PickupAction;

// Makes a new PickupActor action.
::std::unique_ptr<PickupAction> MakePickupAction(const PickupParams &params);

}  // namespace actors
}  // namespace frc971

#endif  // FRC971_ACTORS_PICKUP_ACTOR_H_
