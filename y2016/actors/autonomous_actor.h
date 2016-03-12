#ifndef Y2016_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2016_ACTORS_AUTONOMOUS_ACTOR_H_

#include <memory>

#include "aos/common/actions/actor.h"
#include "aos/common/actions/actions.h"
#include "y2016/actors/autonomous_action.q.h"

namespace y2016 {
namespace actors {

class AutonomousActor
    : public ::aos::common::actions::ActorBase<AutonomousActionQueueGroup> {
 public:
  explicit AutonomousActor(AutonomousActionQueueGroup *s);

  bool RunAction(const actors::AutonomousActionParams &params) override;
};

using AutonomousAction =
    ::aos::common::actions::TypedAction<AutonomousActionQueueGroup>;

// Makes a new AutonomousActor action.
::std::unique_ptr<AutonomousAction> MakeAutonomousAction(
    const ::y2016::actors::AutonomousActionParams &params);

}  // namespace actors
}  // namespace y2016

#endif  // Y2016_ACTORS_AUTONOMOUS_ACTOR_H_
