#include <stdio.h>

#include <memory>

#include "aos/common/util/phased_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"

#include "bot3/autonomous/auto.q.h"
#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/actors/drivetrain_actor.h"

using ::aos::time::Time;
using ::bot3::control_loops::drivetrain_queue;

namespace bot3 {
namespace autonomous {

struct ProfileParams {
  double velocity;
  double acceleration;
};

namespace time = ::aos::time;

bool ShouldExitAuto() {
  ::bot3::autonomous::autonomous.FetchLatest();
  bool ans = !::bot3::autonomous::autonomous->run_auto;
  if (ans) {
    LOG(INFO, "Time to exit auto mode\n");
  }
  return ans;
}

void HandleAuto() {
  ::aos::time::Time start_time = ::aos::time::Time::Now();
  LOG(INFO, "Starting auto mode at %f\n", start_time.ToSeconds());
  ::std::unique_ptr<::frc971::actors::DrivetrainAction> drive;
  LOG(INFO, "Doing nothing in auto.\n");

  if (ShouldExitAuto()) return;
}

}  // namespace autonomous
}  // namespace bot3
