#include "stdio.h"

#include "aos/common/control_loop/Timing.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/messages/RobotState.q.h"
#include "aos/common/logging/logging.h"

#include "bot3/autonomous/auto.q.h"
#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "bot3/control_loops/shooter/shooter_motor.q.h"
#include "frc971/constants.h"

using ::aos::time::Time;

namespace bot3 {
namespace autonomous {

bool ShouldExitAuto() {
  ::bot3::autonomous::autonomous.FetchLatest();
  bool ans = !::bot3::autonomous::autonomous->run_auto;
  if (ans) {
    LOG(INFO, "Time to exit auto mode\n");
  }
  return ans;
}

void SetShooter(double velocity, bool push) {
  LOG(INFO, "Setting shooter velocity to %f\n", velocity);
  control_loops::shooter.goal.MakeWithBuilder()
    .velocity(velocity).push(push).Send();
}

bool ShooterReady() {
  control_loops::shooter.status.FetchLatest();
  return control_loops::shooter.status->ready;
}

// start with N discs in the indexer
void HandleAuto() {
  double shooter_velocity = 250.0;
  while (!ShouldExitAuto()) {
    SetShooter(shooter_velocity, false);
    while (!ShouldExitAuto() && !ShooterReady());
    if (ShouldExitAuto()) return;
    SetShooter(shooter_velocity, true);
    ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.5));
    SetShooter(shooter_velocity, false);
    // Waits because, until we have feedback,
    // the shooter will always think it's ready.
    ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.5));
  }
  return;
}

}  // namespace autonomous
}  // namespace bot3
