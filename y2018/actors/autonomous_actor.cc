#include "y2018/actors/autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2018/control_loops/drivetrain/drivetrain_base.h"

namespace y2018 {
namespace actors {
using ::frc971::ProfileParameters;

using ::frc971::control_loops::drivetrain_queue;
using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;

namespace {

double DoubleSeconds(monotonic_clock::duration duration) {
  return ::std::chrono::duration_cast<::std::chrono::duration<double>>(duration)
      .count();
}

const ProfileParameters kFinalSwitchDrive = {0.5, 0.5};
const ProfileParameters kDrive = {5.0, 2.5};
const ProfileParameters kSlowDrive = {1.5, 2.5};
const ProfileParameters kFarSwitchTurnDrive = {2.0, 2.5};
const ProfileParameters kTurn = {4.0, 2.0};
const ProfileParameters kSweepingTurn = {5.0, 7.0};
const ProfileParameters kFastTurn = {5.0, 7.0};

}  // namespace

AutonomousActor::AutonomousActor(
    ::frc971::autonomous::AutonomousActionQueueGroup *s)
    : frc971::autonomous::BaseAutonomousActor(
          s, control_loops::drivetrain::GetDrivetrainConfig()) {}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams &params) {
  monotonic_clock::time_point start_time = monotonic_clock::now();
  LOG(INFO, "Starting autonomous action with mode %" PRId32 "\n", params.mode);
  Reset();

  switch (params.mode) {
    case 1: {
      constexpr double kDriveDistance = 3.2;
      // Start on the left.   Drive, arc a turn, and drop in the close switch.
      StartDrive(-kDriveDistance, 0.0, kDrive, kTurn);
      if (!WaitForDriveNear(2.0, M_PI / 2.0)) return true;

      // Now, close so let's move the arm up.
      set_arm_goal_position(arm::BackSwitchIndex());
      SendSuperstructureGoal();

      StartDrive(0.0, 0.0, kSlowDrive, kSweepingTurn);
      if (!WaitForDriveNear(1.6, M_PI / 2.0)) return true;

      StartDrive(0.0, -M_PI / 4.0 - 0.2, kSlowDrive, kSweepingTurn);
      if (!WaitForDriveNear(0.2, 0.2)) return true;
      set_max_drivetrain_voltage(6.0);
      ::std::this_thread::sleep_for(chrono::milliseconds(300));

      set_open_claw(true);
      SendSuperstructureGoal();

      ::std::this_thread::sleep_for(chrono::milliseconds(500));
      set_max_drivetrain_voltage(12.0);
      StartDrive(0.2, 0.0, kDrive, kTurn);
      if (!WaitForTurnProfileDone()) return true;

      set_arm_goal_position(arm::NeutralIndex());
      SendSuperstructureGoal();

    } break;
    case 0: {
      // Start on the left.   Hit the scale.
      constexpr double kFullDriveLength = 9.95;
      constexpr double kTurnDistance = 4.40;
      StartDrive(-kFullDriveLength, 0.0, kDrive, kTurn);
      if (!WaitForDriveProfileNear(kFullDriveLength - (kTurnDistance - 1.4)))
        return true;
      StartDrive(0.0, 0.0, kFarSwitchTurnDrive, kTurn);

      if (!WaitForDriveProfileNear(kFullDriveLength - kTurnDistance))
        return true;
      StartDrive(0.0, -M_PI / 2.0, kFarSwitchTurnDrive, kSweepingTurn);
      if (!WaitForTurnProfileDone()) return true;
      StartDrive(0.0, 0.0, kDrive, kTurn);
      if (!WaitForDriveProfileDone()) return true;

      // Now, close so let's move the arm up.
      set_arm_goal_position(arm::FrontSwitchAutoIndex());
      SendSuperstructureGoal();

      StartDrive(0.0, M_PI / 2.0, kDrive, kFastTurn);
      if (!WaitForTurnProfileDone()) return true;

      set_max_drivetrain_voltage(6.0);
      StartDrive(0.35, 0.0, kFinalSwitchDrive, kTurn);
      if (!WaitForArmTrajectoryClose(0.001)) return true;
      //if (!WaitForDriveNear(0.2, M_PI / 2.0)) return true;
      ::std::this_thread::sleep_for(chrono::milliseconds(1500));

      set_open_claw(true);
      SendSuperstructureGoal();

      ::std::this_thread::sleep_for(chrono::milliseconds(1500));
      set_arm_goal_position(arm::NeutralIndex());
      SendSuperstructureGoal();
      if (ShouldCancel()) return true;
      set_max_drivetrain_voltage(12.0);
      StartDrive(-0.5, 0.0, kDrive, kTurn);
      if (!WaitForDriveProfileDone()) return true;
    } break;

    case 3:
    case 2: {
      // Start on the left.   Hit the scale.
      constexpr double kDriveDistance = 7.0;
      // Drive.
      StartDrive(-kDriveDistance, 0.0, kDrive, kTurn);
      if (!WaitForDriveNear(kDriveDistance - 1.0, M_PI / 2.0)) return true;
      // Once we are away from the wall, start the arm.
      set_arm_goal_position(arm::BackHighBoxIndex());
      SendSuperstructureGoal();

      // We are starting to get close. Slow down for the turn.
      if (!WaitForDriveNear(2.5, M_PI / 2.0)) return true;
      StartDrive(0.0, 0.0, kSlowDrive, kSweepingTurn);

      // Once we've gotten slowed down a bit, start turning.
      if (!WaitForDriveNear(1.6, M_PI / 2.0)) return true;
      StartDrive(0.0, -M_PI / 4.0 - 0.1, kSlowDrive, kSweepingTurn);

      // Get close and open the claw.
      if (!WaitForDriveNear(0.25, 0.25)) return true;
      set_open_claw(true);
      SendSuperstructureGoal();

      ::std::this_thread::sleep_for(chrono::milliseconds(500));

      StartDrive(0.25, 0.0, kDrive, kTurn);
      if (!WaitForDriveProfileDone()) return true;
    } break;
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

}  // namespace actors
}  // namespace y2018
