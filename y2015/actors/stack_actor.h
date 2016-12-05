#ifndef Y2015_ACTORS_STACK_ACTOR_H_
#define Y2015_ACTORS_STACK_ACTOR_H_

#include <stdint.h>

#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "y2015/actors/stack_action.q.h"
#include "y2015/actors/fridge_profile_lib.h"

namespace y2015 {
namespace actors {

class StackActor : public FridgeActorBase<StackActionQueueGroup> {
 public:
  explicit StackActor(StackActionQueueGroup *queues);

  bool RunAction(const StackParams &params) override;
};

typedef aos::common::actions::TypedAction<StackActionQueueGroup> StackAction;

// Makes a new stackActor action.
::std::unique_ptr<StackAction> MakeStackAction(const StackParams &params);

}  // namespace actors
}  // namespace y2015

#endif
