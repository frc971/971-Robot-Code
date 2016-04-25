#ifndef Y2016_ACTORS_SUPERSTRUCTURE_ACTOR_H_
#define Y2016_ACTORS_SUPERSTRUCTURE_ACTOR_H_

#include <memory>

#include "aos/common/actions/actor.h"
#include "aos/common/actions/actions.h"
#include "y2016/actors/superstructure_action.q.h"

namespace y2016 {
namespace actors {

class SuperstructureActor
    : public ::aos::common::actions::ActorBase<SuperstructureActionQueueGroup> {
 public:
  explicit SuperstructureActor(SuperstructureActionQueueGroup *s);

  // Internal struct holding superstructure goals sent by autonomous to the
  // loop.
  struct SuperstructureGoal {
    double intake;
    double shoulder;
    double wrist;
  };
  SuperstructureGoal superstructure_goal_;
  bool RunAction(const actors::SuperstructureActionParams &params) override;
  void MoveSuperstructure(double shoulder, double shooter, bool unfold_climber);
  void WaitForSuperstructure();
  bool SuperstructureProfileDone();
  bool SuperstructureDone();
};

using SuperstructureAction =
    ::aos::common::actions::TypedAction<SuperstructureActionQueueGroup>;

// Makes a new SuperstructureActor action.
::std::unique_ptr<SuperstructureAction> MakeSuperstructureAction(
    const ::y2016::actors::SuperstructureActionParams &params);

}  // namespace actors
}  // namespace y2016

#endif  // Y2016_ACTORS_SUPERSTRUCTURE_ACTOR_H_
