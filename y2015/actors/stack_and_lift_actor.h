#ifndef Y2015_ACTORS_STACK_AND_LIFT_ACTOR_H_
#define Y2015_ACTORS_STACK_AND_LIFT_ACTOR_H_

#include <stdint.h>

#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "y2015/actors/stack_and_lift_action.q.h"
#include "y2015/actors/fridge_profile_lib.h"

namespace y2015 {
namespace actors {

class StackAndLiftActor : public FridgeActorBase<StackAndLiftActionQueueGroup> {
 public:
  explicit StackAndLiftActor(StackAndLiftActionQueueGroup *queues);

  bool RunAction(const StackAndLiftParams &params) override;
};

typedef aos::common::actions::TypedAction<StackAndLiftActionQueueGroup>
    StackAndLiftAction;

// Makes a new stackActor action.
::std::unique_ptr<StackAndLiftAction> MakeStackAndLiftAction(
    const StackAndLiftParams &params);

}  // namespace actors
}  // namespace y2015

#endif  // Y2015_ACTORS_STACK_AND_LIFT_ACTOR_H_
