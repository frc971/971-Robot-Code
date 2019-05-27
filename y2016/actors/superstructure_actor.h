#ifndef Y2016_ACTORS_SUPERSTRUCTURE_ACTOR_H_
#define Y2016_ACTORS_SUPERSTRUCTURE_ACTOR_H_

#include <memory>

#include "aos/actions/actor.h"
#include "aos/actions/actions.h"
#include "y2016/actors/superstructure_action.q.h"

namespace y2016 {
namespace actors {

class SuperstructureActor
    : public ::aos::common::actions::ActorBase<SuperstructureActionQueueGroup> {
 public:
  typedef ::aos::common::actions::TypedActionFactory<
      SuperstructureActionQueueGroup>
      Factory;

  explicit SuperstructureActor(::aos::EventLoop *event_loop);

  static Factory MakeFactory(::aos::EventLoop *event_loop) {
    return Factory(event_loop, ".y2016.actors.superstructure_action");
  }

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

}  // namespace actors
}  // namespace y2016

#endif  // Y2016_ACTORS_SUPERSTRUCTURE_ACTOR_H_
