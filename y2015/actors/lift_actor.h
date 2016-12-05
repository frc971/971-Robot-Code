#ifndef Y2015_ACTORS_LIFT_ACTOR_H_
#define Y2015_ACTORS_LIFT_ACTOR_H_

#include <stdint.h>

#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "y2015/actors/lift_action.q.h"
#include "y2015/actors/fridge_profile_lib.h"

namespace y2015 {
namespace actors {

class LiftActor : public FridgeActorBase<LiftActionQueueGroup> {
 public:
  explicit LiftActor(LiftActionQueueGroup *queues);

  bool RunAction(const LiftParams &params) override;
};

typedef aos::common::actions::TypedAction<LiftActionQueueGroup> LiftAction;

// Makes a new LiftActor action.
::std::unique_ptr<LiftAction> MakeLiftAction(const LiftParams &params);

}  // namespace actors
}  // namespace y2015

#endif  // Y2015_ACTORS_LIFT_ACTOR_H_
