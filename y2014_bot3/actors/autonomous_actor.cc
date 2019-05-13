#include "y2014_bot3/actors/autonomous_actor.h"

#include <inttypes.h>
#include <chrono>
#include <cmath>

#include "aos/events/event-loop.h"
#include "aos/logging/logging.h"
#include "aos/util/phased_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2014_bot3/control_loops/drivetrain/drivetrain_base.h"

namespace y2014_bot3 {
namespace actors {
using ::frc971::ProfileParameters;

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;

namespace {

double DoubleSeconds(monotonic_clock::duration duration) {
  return ::std::chrono::duration_cast<::std::chrono::duration<double>>(duration)
      .count();
}

const ProfileParameters kDrive = {5.0, 2.5};
const ProfileParameters kTurn = {8.0, 3.0};

}  // namespace

AutonomousActor::AutonomousActor(
    ::aos::EventLoop *event_loop,
    ::frc971::autonomous::AutonomousActionQueueGroup *s)
    : frc971::autonomous::BaseAutonomousActor(
          event_loop, s, control_loops::drivetrain::GetDrivetrainConfig()) {}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams &params) {
  monotonic_clock::time_point start_time = monotonic_clock::now();
  LOG(INFO, "Starting autonomous action with mode %" PRId32 "\n", params.mode);
  Reset();

  StartDrive(1.0, 0.0, kDrive, kTurn);
  if (!WaitForDriveDone()) return true;

  LOG(INFO, "Done %f\n", DoubleSeconds(monotonic_clock::now() - start_time));

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (!ShouldCancel()) {
    phased_loop.SleepUntilNext();
  }
  LOG(DEBUG, "Done running\n");

  return true;
}

}  // namespace actors
}  // namespace y2014_bot3
