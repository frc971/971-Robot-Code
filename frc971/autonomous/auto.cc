#include "stdio.h"

#include "aos/aos_core.h"
#include "aos/common/control_loop/Timing.h"
#include "aos/common/time.h"
#include "frc971/autonomous/auto.q.h"
#include "frc971/control_loops/DriveTrain.q.h"
#include "frc971/control_loops/wrist/wrist_motor.q.h"
#include "frc971/control_loops/index/index_motor.q.h"
#include "frc971/control_loops/shooter/shooter_motor.q.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor.q.h"

using ::aos::time::Time;

namespace frc971 {
namespace autonomous {

void SetShooterVelocity(double velocity) {
  control_loops::shooter.goal.MakeWithBuilder()
    .velocity(velocity).Send();
}

void StartIntake() {
  control_loops::intake.goal.MakeWithBuilder()
    .goal_state(2).Send();
}

void PreloadIntake() {
  control_loops::intake.goal.MakeWithBuilder()
    .goal_state(3).Send();
}

void ShootIntake() {
  control_loops::intake.goal.MakeWithBuilder()
    .goal_state(4).Send();
}

void WaitForShooter() {
  control_loops::shooter.status.FetchLatest();
  while (!control_loops::shooter.status->ready) {
    control_loops::shooter.status.FetchNextBlocking();
  }
}

// TODO(aschuh): Send the number of discs shot in the status message.
void ShootNDiscs(int n) {
  control_loops::intake.status.FetchLatest();
  while (!control_loops::intake.status->ready) {
    control_loops::intake.status.FetchNextBlocking();
  }
}

bool ShouldExitAuto() {
  ::frc971::autonomous::autonomous.FetchLatest();
  return !::frc971::autonomous::autonomous->run_auto;
}

// TODO(aschuh): Drivetrain needs a profile somewhere.
// Here isn't the best spot.  It should be in another thread/process

void HandleAuto() {
  SetShooterVelocity(200.0);
  PreloadIntake();

  WaitForShooter();
  ShootIntake();

  // Wait for the wrist and angle adjust to finish zeroing before shooting.
  // Sigh, constants go where?

  control_loops::drivetrain.goal.MakeWithBuilder()
    .control_loop_driving(true)
    .left_goal(0.0)
    .right_goal(0.0).Send();
}

}  // namespace autonomous
}  // namespace frc971
