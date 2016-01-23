#include "y2016/actors/superstructure_actor.h"

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"
#include "y2016/actors/superstructure_actor.h"
#include "y2016/control_loops/superstructure/superstructure.q.h"

namespace y2016 {
namespace actors {

SuperstructureActor::SuperstructureActor(
    actors::SuperstructureActionQueueGroup* s)
    : aos::common::actions::ActorBase<actors::SuperstructureActionQueueGroup>(
          s) {}

bool SuperstructureActor::RunAction(
    const actors::SuperstructureActionParams& params) {
  LOG(INFO, "Starting superstructure action with value %f", params.value);

  while (true) {
    control_loops::superstructure_queue.status.FetchLatest();
    ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(5),
                                       ::aos::time::Time::InMS(5) / 2);
    break;
  }

  return true;
}

::std::unique_ptr<SuperstructureAction> MakeSuperstructureAction(
    const ::y2016::actors::SuperstructureActionParams& params) {
  return ::std::unique_ptr<SuperstructureAction>(new SuperstructureAction(
      &::y2016::actors::superstructure_action, params));
}

}  // namespace actors
}  // namespace y2016
