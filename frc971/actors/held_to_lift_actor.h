#ifndef FRC971_ACTORS_HELD_TO_LIFT_ACTOR_H_
#define FRC971_ACTORS_HELD_TO_LIFT_ACTOR_H_

#include <stdint.h>

#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "frc971/actors/held_to_lift_action.q.h"
#include "frc971/actors/fridge_profile_lib.h"

namespace frc971 {
namespace actors {

class HeldToLiftActor : public FridgeActorBase<HeldToLiftActionQueueGroup> {
 public:
  explicit HeldToLiftActor(HeldToLiftActionQueueGroup *queues);

  bool RunAction(const HeldToLiftParams &params) override;
};

typedef aos::common::actions::TypedAction<HeldToLiftActionQueueGroup>
    HeldToLiftAction;

// Makes a new HeldToLiftActor action.
::std::unique_ptr<HeldToLiftAction> MakeHeldToLiftAction(
    const HeldToLiftParams &params);

}  // namespace actors
}  // namespace frc971

#endif  // FRC971_ACTORS_HELD_TO_LIFT_ACTOR_H_
