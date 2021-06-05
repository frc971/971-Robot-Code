#include "y2021_bot3/actors/autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/logging/logging.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2021_bot3/control_loops/drivetrain/drivetrain_base.h"

namespace y2021_bot3 {
namespace actors {

using ::aos::monotonic_clock;
using ::frc971::ProfileParametersT;
using frc971::control_loops::drivetrain::LocalizerControl;
namespace chrono = ::std::chrono;

AutonomousActor::AutonomousActor(::aos::EventLoop *event_loop)
    : frc971::autonomous::BaseAutonomousActor(
          event_loop, control_loops::drivetrain::GetDrivetrainConfig()) {}

void AutonomousActor::Reset() {
  InitializeEncoders();
  ResetDrivetrain();
}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  Reset();

  AOS_LOG(INFO, "Params are %d\n", params->mode());
  return true;
}

}  // namespace actors
}  // namespace y2021_bot3
