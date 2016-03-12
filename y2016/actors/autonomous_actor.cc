#include "y2016/actors/autonomous_actor.h"

#include <inttypes.h>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"
#include "y2016/actors/autonomous_action.q.h"

namespace y2016 {
namespace actors {

AutonomousActor::AutonomousActor(actors::AutonomousActionQueueGroup *s)
    : aos::common::actions::ActorBase<actors::AutonomousActionQueueGroup>(s) {}

bool AutonomousActor::RunAction(const actors::AutonomousActionParams &params) {
  LOG(INFO, "Starting autonomous action with mode %" PRId32 "\n", params.mode);

  while (true) {
    ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(5),
                                        ::aos::time::Time::InMS(5) / 2);
    break;
  }

  return true;
}

::std::unique_ptr<AutonomousAction> MakeAutonomousAction(
    const ::y2016::actors::AutonomousActionParams &params) {
  return ::std::unique_ptr<AutonomousAction>(
      new AutonomousAction(&::y2016::actors::autonomous_action, params));
}

}  // namespace actors
}  // namespace y2016
