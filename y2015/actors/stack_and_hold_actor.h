#ifndef Y2015_ACTORS_STACK_AND_HOLD_ACTOR_H_
#define Y2015_ACTORS_STACK_AND_HOLD_ACTOR_H_

#include <stdint.h>

#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "y2015/actors/stack_and_hold_action.q.h"
#include "y2015/actors/fridge_profile_lib.h"

namespace y2015 {
namespace actors {

class StackAndHoldActor : public FridgeActorBase<StackAndHoldActionQueueGroup> {
 public:
  explicit StackAndHoldActor(StackAndHoldActionQueueGroup *queues);

  bool RunAction(const StackAndHoldParams &params) override;
};

typedef aos::common::actions::TypedAction<StackAndHoldActionQueueGroup>
    StackAndHoldAction;

// Makes a new stackActor action.
::std::unique_ptr<StackAndHoldAction> MakeStackAndHoldAction(
    const StackAndHoldParams &params);

}  // namespace actors
}  // namespace y2015

#endif  // Y2015_ACTORS_STACK_AND_HOLD_ACTOR_H_
