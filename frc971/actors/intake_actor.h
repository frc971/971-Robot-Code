#ifndef FRC971_ACTORS_INTAKE_ACTOR_H_
#define FRC971_ACTORS_INTAKE_ACTOR_H_

#include <stdint.h>

#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "frc971/actors/claw_actor.h"
#include "frc971/actors/fridge_profile_actor.h"
#include "frc971/actors/intake_action.q.h"

namespace frc971 {
namespace actors {

class IntakeActor
    : public aos::common::actions::ActorBase<IntakeActionQueueGroup> {
 public:
  explicit IntakeActor(IntakeActionQueueGroup *queues);

  bool RunAction(const IntakeParams &params) override;

 private:
  // Goes and waits for the claw and fridge actions to complete, handling
  // cancellation properly.
  // fridge: The fridge action to wait on, or nullptr if we don't want to wait
  // for the fridge.
  // claw: The claw action to wait on, or nullptr if we don't want to wait for
  // the claw.
  void WaitForSystems(FridgeAction *fridge, ClawAction *claw);
};

typedef aos::common::actions::TypedAction<IntakeActionQueueGroup> IntakeAction;

// Makes a new IntakeActor action.
::std::unique_ptr<IntakeAction> MakeIntakeAction(const IntakeParams &params);

}  // namespace actors
}  // namespace frc971

#endif
