#ifndef Y2017_ACTORS_AUTONOMOUS_ACTOR_H_
#define Y2017_ACTORS_AUTONOMOUS_ACTOR_H_

#include <chrono>
#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/actions/actor.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "y2017/actors/autonomous_action.q.h"

namespace y2017 {
namespace actors {
using ::frc971::ProfileParameters;

class AutonomousActor
    : public ::aos::common::actions::ActorBase<AutonomousActionQueueGroup> {
 public:
  explicit AutonomousActor(AutonomousActionQueueGroup *s);

  bool RunAction(const actors::AutonomousActionParams &params) override;
 private:
  void WaitUntilDoneOrCanceled(::std::unique_ptr<aos::common::actions::Action>
      action);
};

typedef ::aos::common::actions::TypedAction<AutonomousActionQueueGroup>
    AutonomousAction;

// Makes a new AutonomousActor action.
::std::unique_ptr<AutonomousAction> MakeAutonomousAction(
    const ::y2017::actors::AutonomousActionParams &params);

}  // namespace actors
}  // namespace y2017

#endif  // Y2017_ACTORS_AUTONOMOUS_ACTOR_H_
