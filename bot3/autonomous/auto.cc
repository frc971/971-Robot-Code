#include "stdio.h"

#include "aos/common/control_loop/Timing.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/messages/RobotState.q.h"
#include "aos/common/logging/logging.h"

#include "bot3/autonomous/auto.q.h"
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
  control_loops::shooter.goal.MakeWithBuilder().intake(0)
    .velocity(velocity).push(push).Send();
}

void SpinUp() {
  LOG(INFO, "Tricking shooter into running at full power...\n");
  control_loops::shooter.position.MakeWithBuilder().velocity(0).Send();
}

bool ShooterReady() {
  bool ready = control_loops::shooter.status.FetchNextBlocking() && control_loops::shooter.status->ready;
  LOG(DEBUG, "Shooter ready: %d\n", ready);
  return ready;
}

void HandleAuto() {
  double shooter_velocity = 500.0;
  bool first_accl = true;
  while (!ShouldExitAuto()) {
    SetShooter(shooter_velocity, false);
    if (first_accl) {
      ::aos::time::SleepFor(::aos::time::Time::InSeconds(3.0));
      first_accl = false;
    }
    if (ShouldExitAuto()) return;
    SetShooter(shooter_velocity, true);
    ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.25));
    SetShooter(shooter_velocity, false);
    // We just shot, trick it into spinning back up.
    SpinUp();
    ::aos::time::SleepFor(::aos::time::Time::InSeconds(2.0));
  }
  return;
}

}  // namespace autonomous
}  // namespace bot3
