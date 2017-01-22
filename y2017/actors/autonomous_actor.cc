#include "y2017/actors/autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2017/actors/autonomous_action.q.h"

namespace y2017 {
namespace actors {
using ::frc971::control_loops::drivetrain_queue;
using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;

namespace {
double DoubleSeconds(monotonic_clock::duration duration) {
  return ::std::chrono::duration_cast<::std::chrono::duration<double>>(duration)
      .count();
}
}  // namespace

AutonomousActor::AutonomousActor(actors::AutonomousActionQueueGroup *s)
    : aos::common::actions::ActorBase<actors::AutonomousActionQueueGroup>(s) {}

void AutonomousActor::WaitUntilDoneOrCanceled(
    ::std::unique_ptr<aos::common::actions::Action> action) {
  if (!action) {
    LOG(ERROR, "No action, not waiting\n");
    return;
  }

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    // Poll the running bit and see if we should cancel.
    phased_loop.SleepUntilNext();
    if (!action->Running() || ShouldCancel()) {
      return;
    }
  }
}

bool AutonomousActor::RunAction(const actors::AutonomousActionParams &params) {
  monotonic_clock::time_point start_time = monotonic_clock::now();
  LOG(INFO, "Starting autonomous action with mode %" PRId32 "\n", params.mode);

  switch (params.mode) {
    case 0:
      break;

    default:
      LOG(ERROR, "Invalid auto mode %d\n", params.mode);
      return true;
  }

  LOG(INFO, "Done %f\n", DoubleSeconds(monotonic_clock::now() - start_time));

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);

  while (!ShouldCancel()) {
    phased_loop.SleepUntilNext();
  }
  LOG(DEBUG, "Done running\n");

  return true;
}

::std::unique_ptr<AutonomousAction> MakeAutonomousAction(
    const ::y2017::actors::AutonomousActionParams &params) {
  return ::std::unique_ptr<AutonomousAction>(
      new AutonomousAction(&::y2017::actors::autonomous_action, params));
}

}  // namespace actors
}  // namespace y2017
