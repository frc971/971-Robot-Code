#ifndef Y2015_ACTORS_PICKUP_ACTOR_H_
#define Y2015_ACTORS_PICKUP_ACTOR_H_

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "y2015/actors/pickup_action.q.h"

namespace y2015 {
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
}  // namespace y2015

#endif  // Y2015_ACTORS_PICKUP_ACTOR_H_
