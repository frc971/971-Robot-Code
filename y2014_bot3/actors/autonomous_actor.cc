#include "y2014_bot3/actors/autonomous_actor.h"

#include <inttypes.h>
#include <chrono>
#include <cmath>

#include "aos/events/event_loop.h"
#include "aos/logging/logging.h"
#include "aos/util/phased_loop.h"
#include "y2014_bot3/control_loops/drivetrain/drivetrain_base.h"

namespace y2014_bot3 {
namespace actors {
using ::frc971::ProfileParametersT;

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;

namespace {

ProfileParametersT MakeProfileParameters(float max_velocity,
                                         float max_acceleration) {
  ProfileParametersT result;
  result.max_velocity = max_velocity;
  result.max_acceleration = max_acceleration;
  return result;
}

const ProfileParametersT kDrive = MakeProfileParameters(5.0, 2.5);
const ProfileParametersT kTurn = MakeProfileParameters(8.0, 3.0);

}  // namespace

AutonomousActor::AutonomousActor(::aos::EventLoop *event_loop)
    : frc971::autonomous::BaseAutonomousActor(
          event_loop, control_loops::drivetrain::GetDrivetrainConfig()) {}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  const monotonic_clock::time_point start_time = monotonic_now();
  AOS_LOG(INFO, "Starting autonomous action with mode %" PRId32 "\n",
          params->mode());
  Reset();

  StartDrive(1.0, 0.0, kDrive, kTurn);
  if (!WaitForDriveDone()) return true;

  AOS_LOG(INFO, "Done %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);
  while (!ShouldCancel()) {
    phased_loop.SleepUntilNext();
  }
  AOS_LOG(DEBUG, "Done running\n");

  return true;
}

}  // namespace actors
}  // namespace y2014_bot3
