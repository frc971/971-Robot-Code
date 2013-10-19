#include "stdio.h"

#include "aos/common/control_loop/Timing.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/messages/RobotState.q.h"
#include "aos/common/logging/logging.h"

#include "bot3/autonomous/auto.q.h"
#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/constants.h"

using ::aos::time::Time;

namespace bot3 {
namespace autonomous {

// start with N discs in the indexer
void HandleAuto() {
  // TODO (danielp): Do something in auto.
  // (That's why I left all the includes.)
  LOG(INFO, "Auto mode is not currently implemented on this robot.\n");
}

}  // namespace autonomous
}  // namespace bot3
