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
const ProfileParameters kThirdBoxDrive = {3.0, 2.5};
const ProfileParameters kSlowDrive = {1.5, 2.5};
const ProfileParameters kScaleTurnDrive = {3.0, 2.5};
const ProfileParameters kFarSwitchTurnDrive = {2.0, 2.5};
const ProfileParameters kTurn = {4.0, 2.0};
const ProfileParameters kSweepingTurn = {5.0, 7.0};
const ProfileParameters kFastTurn = {5.0, 7.0};
const ProfileParameters kReallyFastTurn = {1.5, 9.0};

const ProfileParameters kThirdBoxSlowBackup = {0.35, 1.5};
const ProfileParameters kThirdBoxSlowTurn = {1.5, 4.0};

const ProfileParameters kThirdBoxPlaceDrive = {4.0, 2.3};

const ProfileParameters kFinalFrontFarSwitchDrive = {2.0, 2.0};

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
      StartDrive(-3.2, 0.0, kDrive, kTurn);
      if (!WaitForDriveProfileDone()) return true;
    } break;
    case 200:
    {
      constexpr bool kDriveBehind = true;
      if (kDriveBehind) {
        // Start on the left.   Hit the switch.
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
        // if (!WaitForDriveNear(0.2, M_PI / 2.0)) return true;
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
      } else {
        // Start on the left.   Hit the switch.
        constexpr double kFullDriveLength = 5.55;
        constexpr double kTurnDistance = 0.35;
        StartDrive(-kFullDriveLength, 0.0, kFarSwitchTurnDrive, kTurn);

        if (!WaitForDriveProfileNear(kFullDriveLength - kTurnDistance))
          return true;
        StartDrive(0.0, -M_PI / 2.0, kFarSwitchTurnDrive, kSweepingTurn);
        if (!WaitForTurnProfileDone()) return true;
        StartDrive(0.0, 0.0, kDrive, kTurn);
        if (!WaitForDriveProfileDone()) return true;

        // Now, close so let's move the arm up.
        set_arm_goal_position(arm::FrontSwitchIndex());
        SendSuperstructureGoal();

        StartDrive(0.0, -M_PI / 2.0, kDrive, kFastTurn);
        if (!WaitForTurnProfileDone()) return true;

        set_max_drivetrain_voltage(10.0);
        StartDrive(1.1, 0.0, kDrive, kTurn);
        if (!WaitForArmTrajectoryClose(0.001)) return true;
        if (!WaitForDriveNear(0.6, M_PI / 2.0)) return true;
        StartDrive(0.0, 0.0, kFinalFrontFarSwitchDrive, kTurn);

        if (!WaitForDriveNear(0.3, M_PI / 2.0)) return true;
        set_max_drivetrain_voltage(6.0);
        StartDrive(0.0, 0.0, kFinalFrontFarSwitchDrive, kTurn);

        // if (!WaitForDriveNear(0.2, M_PI / 2.0)) return true;
        ::std::this_thread::sleep_for(chrono::milliseconds(300));

        set_open_claw(true);
        SendSuperstructureGoal();

        ::std::this_thread::sleep_for(chrono::milliseconds(1000));
        set_arm_goal_position(arm::NeutralIndex());
        SendSuperstructureGoal();
        if (ShouldCancel()) return true;
        set_max_drivetrain_voltage(12.0);
        StartDrive(-0.5, 0.0, kDrive, kTurn);
        if (!WaitForDriveProfileDone()) return true;
      }
    } break;

    case 3:
    case 2: {
      // Start on the left.   Hit the scale.
      constexpr double kDriveDistance = 6.95;
      // Distance and angle to do the big drive to the third cube.
      constexpr double kThirdCubeDrive = 1.57;
      constexpr double kThirdCubeTurn = M_PI / 4.0 - 0.1;
      // Angle to do the slow pickup turn on the third box.
      constexpr double kThirdBoxEndTurnAngle = 0.20;

      // Distance to drive back to the scale with the third cube.
      constexpr double kThirdCubeDropDrive = kThirdCubeDrive + 0.30;

      // Drive.
      StartDrive(-kDriveDistance, 0.0, kDrive, kTurn);
      if (!WaitForDriveNear(kDriveDistance - 1.0, M_PI / 2.0)) return true;
      // Once we are away from the wall, start the arm.
      set_arm_goal_position(arm::BackMiddle2BoxIndex());
      SendSuperstructureGoal();

      // We are starting to get close. Slow down for the turn.
      if (!WaitForDriveNear(4.0, M_PI / 2.0)) return true;
      StartDrive(0.0, 0.0, kScaleTurnDrive, kFastTurn);

      // Once we've gotten slowed down a bit, start turning.
      if (!WaitForDriveNear(3.25, M_PI / 2.0)) return true;
      StartDrive(0.0, -M_PI / 6.0, kScaleTurnDrive, kFastTurn);

      if (!WaitForDriveNear(1.0, M_PI / 2.0)) return true;
      StartDrive(0.0, M_PI / 6.0, kScaleTurnDrive, kFastTurn);

      // Get close and open the claw.
      if (!WaitForDriveNear(0.15, 0.25)) return true;
      set_open_claw(true);
      SendSuperstructureGoal();
      set_intake_angle(-0.40);
      LOG(INFO, "Dropped first box %f\n",
          DoubleSeconds(monotonic_clock::now() - start_time));

      ::std::this_thread::sleep_for(chrono::milliseconds(500));

      set_grab_box(true);
      SendSuperstructureGoal();

      ::std::this_thread::sleep_for(chrono::milliseconds(200));

      LOG(INFO, "Starting second box drive %f\n",
          DoubleSeconds(monotonic_clock::now() - start_time));
      constexpr double kSecondBoxSwerveAngle = 0.35;
      constexpr double kSecondBoxDrive = 1.43;
      StartDrive(kSecondBoxDrive, 0.0, kDrive, kFastTurn);
      if (!WaitForDriveNear(kSecondBoxDrive - 0.2, M_PI / 2.0)) return true;

      StartDrive(0.0, kSecondBoxSwerveAngle, kDrive, kFastTurn);
      if (!WaitForDriveNear(0.5, M_PI / 2.0)) return true;

      set_open_claw(true);
      set_disable_box_correct(false);
      SendSuperstructureGoal();

      StartDrive(0.0, -kSecondBoxSwerveAngle, kDrive, kFastTurn);

      if (!WaitForDriveProfileDone()) return true;
      set_intake_angle(0.10);
      set_arm_goal_position(arm::BackHighBoxIndex());
      set_open_claw(false);

      set_roller_voltage(10.0);
      SendSuperstructureGoal();

      LOG(INFO, "Grabbing second box %f\n",
          DoubleSeconds(monotonic_clock::now() - start_time));
      if (!WaitForBoxGrabed()) return true;

      LOG(INFO, "Got second box %f\n",
          DoubleSeconds(monotonic_clock::now() - start_time));
      ::std::this_thread::sleep_for(chrono::milliseconds(600));

      set_grab_box(false);
      set_arm_goal_position(arm::UpIndex());
      set_roller_voltage(0.0);
      set_disable_box_correct(false);
      SendSuperstructureGoal();
      LOG(INFO, "Driving to place second box %f\n",
          DoubleSeconds(monotonic_clock::now() - start_time));

      StartDrive(-kSecondBoxDrive + 0.2, kSecondBoxSwerveAngle, kDrive,
                 kFastTurn);
      if (!WaitForDriveNear(0.4, M_PI / 2.0)) return true;

      constexpr double kSecondBoxEndExtraAngle = 0.3;
      StartDrive(0.0, -kSecondBoxSwerveAngle - kSecondBoxEndExtraAngle, kDrive,
                 kFastTurn);

      LOG(INFO, "Starting throw %f\n",
          DoubleSeconds(monotonic_clock::now() - start_time));
      if (!WaitForDriveNear(0.4, M_PI / 2.0)) return true;
      set_arm_goal_position(arm::BackHighBoxIndex());
      SendSuperstructureGoal();

      // Throw the box.
      if (!WaitForArmTrajectoryClose(0.03)) return true;

      set_open_claw(true);
      set_intake_angle(-M_PI / 4.0);
      LOG(INFO, "Releasing second box %f\n",
          DoubleSeconds(monotonic_clock::now() - start_time));
      SendSuperstructureGoal();

      ::std::this_thread::sleep_for(chrono::milliseconds(700));
      set_open_claw(false);
      set_grab_box(true);
      SendSuperstructureGoal();

      LOG(INFO, "Driving to third box %f\n",
          DoubleSeconds(monotonic_clock::now() - start_time));
      StartDrive(kThirdCubeDrive, kSecondBoxEndExtraAngle, kThirdBoxDrive,
                 kFastTurn);
      if (!WaitForDriveNear(kThirdCubeDrive - 0.1, M_PI / 4.0)) return true;

      StartDrive(0.0, kThirdCubeTurn, kThirdBoxDrive, kFastTurn);
      if (!WaitForDriveNear(0.3, M_PI / 4.0)) return true;

      set_intake_angle(0.05);
      set_roller_voltage(9.0);
      SendSuperstructureGoal();

      if (!WaitForDriveProfileDone()) return true;
      if (!WaitForTurnProfileDone()) return true;
      StartDrive(0.30, kThirdBoxEndTurnAngle, kThirdBoxSlowBackup, kThirdBoxSlowTurn);
      if (!WaitForDriveProfileDone()) return true;

      if (!WaitForBoxGrabed()) return true;
      LOG(INFO, "Third box grabbed %f\n",
          DoubleSeconds(monotonic_clock::now() - start_time));
      const bool too_late =
          monotonic_clock::now() > start_time + chrono::milliseconds(12000);
      if (too_late) {
        LOG(INFO, "Third box too long, going up. %f\n",
            DoubleSeconds(monotonic_clock::now() - start_time));
        set_grab_box(false);
        set_arm_goal_position(arm::UpIndex());
        set_roller_voltage(0.0);
        SendSuperstructureGoal();
      }
      ::std::this_thread::sleep_for(chrono::milliseconds(600));

      set_grab_box(false);
      if (!too_late) {
        set_arm_goal_position(arm::BackMiddle2BoxIndex());
      }
      set_roller_voltage(0.0);
      SendSuperstructureGoal();

      StartDrive(-kThirdCubeDropDrive, 0.0,
                 kThirdBoxPlaceDrive, kReallyFastTurn);

      if (!WaitForDriveNear(1.40, M_PI / 4.0)) return true;
      StartDrive(0.0, -kThirdCubeTurn - 0.2 - kThirdBoxEndTurnAngle - 0.3,
                 kThirdBoxPlaceDrive, kReallyFastTurn);

      if (!WaitForArmTrajectoryClose(0.005)) return true;
      if (!WaitForDriveProfileDone()) return true;
      if (!WaitForTurnProfileDone()) return true;

      if (!too_late) {
        set_open_claw(true);
        set_intake_angle(-M_PI / 4.0);
        set_roller_voltage(0.0);
        SendSuperstructureGoal();

        LOG(INFO, "Final open %f\n",
            DoubleSeconds(monotonic_clock::now() - start_time));
      }

      ::std::this_thread::sleep_for(chrono::milliseconds(14750) -
                                    (monotonic_clock::now() - start_time));

      set_arm_goal_position(arm::UpIndex());
      SendSuperstructureGoal();

      ::std::this_thread::sleep_for(chrono::milliseconds(15000) -
                                    (monotonic_clock::now() - start_time));

      set_close_claw(true);
      SendSuperstructureGoal();

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
