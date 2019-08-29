#ifndef Y2016_ACTORS_SUPERSTRUCTURE_ACTOR_H_
#define Y2016_ACTORS_SUPERSTRUCTURE_ACTOR_H_

#include <memory>

#include "aos/actions/actions.h"
#include "aos/actions/actor.h"
#include "y2016/actors/superstructure_action_generated.h"
#include "y2016/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2016/control_loops/superstructure/superstructure_status_generated.h"

namespace y2016 {
namespace actors {

class SuperstructureActor
    : public ::aos::common::actions::ActorBase<superstructure_action::Goal> {
 public:
  typedef ::aos::common::actions::TypedActionFactory<
      superstructure_action::Goal>
      Factory;

  explicit SuperstructureActor(::aos::EventLoop *event_loop);

  static Factory MakeFactory(::aos::EventLoop *event_loop) {
    return Factory(event_loop, "/superstructure_action");
  }

 private:
  ::aos::Sender<::y2016::control_loops::superstructure::Goal>
      superstructure_goal_sender_;
  ::aos::Fetcher<::y2016::control_loops::superstructure::Status>
      superstructure_status_fetcher_;
  // Internal struct holding superstructure goals sent by autonomous to the
  // loop.
  struct SuperstructureGoal {
    double intake;
    double shoulder;
    double wrist;
  };
  SuperstructureGoal superstructure_goal_;
  bool RunAction(
      const superstructure_action::SuperstructureActionParams *params) override;
  void MoveSuperstructure(double shoulder, double shooter, bool unfold_climber);
  void WaitForSuperstructure();
  bool SuperstructureProfileDone();
  bool SuperstructureDone();
};

}  // namespace actors
}  // namespace y2016

#endif  // Y2016_ACTORS_SUPERSTRUCTURE_ACTOR_H_
