#ifndef FRC971_ACTORS_INTAKE_ACTOR_H_
#define FRC971_ACTORS_INTAKE_ACTOR_H_

#include <stdint.h>

#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "frc971/actors/intake_action.q.h"

namespace frc971 {
namespace actors {

class IntakeActor
    : public aos::common::actions::ActorBase<IntakeActionQueueGroup> {
 public:
  explicit IntakeActor(IntakeActionQueueGroup *queues);

  bool RunAction(const IntakeParams &params) override;
};

typedef aos::common::actions::TypedAction<IntakeActionQueueGroup> IntakeAction;

// Makes a new IntakeActor action.
::std::unique_ptr<IntakeAction> MakeIntakeAction(const IntakeParams &params);

}  // namespace actors
}  // namespace frc971

#endif
