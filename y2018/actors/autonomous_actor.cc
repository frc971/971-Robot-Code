#include "y2018/actors/autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/logging/logging.h"
#include "aos/util/phased_loop.h"

#include "y2018/control_loops/drivetrain/drivetrain_base.h"

namespace y2018 {
namespace actors {
using ::frc971::ProfileParametersT;

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;

namespace {

namespace arm = ::y2018::control_loops::superstructure::arm;

ProfileParametersT MakeProfileParameters(float max_velocity,
                                         float max_acceleration) {
  ProfileParametersT result;
  result.max_velocity = max_velocity;
  result.max_acceleration = max_acceleration;
  return result;
}

const ProfileParametersT kFinalSwitchDrive = MakeProfileParameters(0.5, 1.5);
const ProfileParametersT kDrive = MakeProfileParameters(5.0, 2.5);
const ProfileParametersT kThirdBoxDrive = MakeProfileParameters(3.0, 2.5);
const ProfileParametersT kSlowDrive = MakeProfileParameters(1.5, 2.5);
const ProfileParametersT kScaleTurnDrive = MakeProfileParameters(3.0, 2.5);
const ProfileParametersT kFarSwitchTurnDrive = MakeProfileParameters(2.0, 2.5);
const ProfileParametersT kFarScaleFinalTurnDrive = kFarSwitchTurnDrive;
const ProfileParametersT kTurn = MakeProfileParameters(4.0, 2.0);
const ProfileParametersT kSweepingTurn = MakeProfileParameters(5.0, 7.0);
const ProfileParametersT kFarScaleSweepingTurn = kSweepingTurn;
const ProfileParametersT kFastTurn = MakeProfileParameters(5.0, 7.0);
const ProfileParametersT kReallyFastTurn = MakeProfileParameters(1.5, 9.0);

const ProfileParametersT kThirdBoxSlowBackup = MakeProfileParameters(0.35, 1.5);
const ProfileParametersT kThirdBoxSlowTurn = MakeProfileParameters(1.5, 4.0);

const ProfileParametersT kThirdBoxPlaceDrive = MakeProfileParameters(4.0, 2.0);

const ProfileParametersT kFinalFrontFarSwitchDrive =
    MakeProfileParameters(2.0, 2.0);

}  // namespace

AutonomousActor::AutonomousActor(::aos::EventLoop *event_loop)
    : frc971::autonomous::BaseAutonomousActor(
          event_loop, control_loops::drivetrain::GetDrivetrainConfig()),
      superstructure_goal_sender_(
          event_loop->MakeSender<::y2018::control_loops::superstructure::Goal>(
              "/superstructure")),
      superstructure_status_fetcher_(
          event_loop
              ->MakeFetcher<::y2018::control_loops::superstructure::Status>(
                  "/superstructure")) {}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  const monotonic_clock::time_point start_time = monotonic_now();
  AOS_LOG(INFO, "Starting autonomous action with mode %" PRId32 "\n",
          params->mode());
  Reset();

  // Switch
  /*
  switch (params->mode()) {
    case 0:
    case 2: {
      if (FarSwitch(start_time, true)) return true;
    } break;

    case 3:
    case 1: {
      if (CloseSwitch(start_time)) return true;
    } break;
  }
  */
  // Scale
  switch (params->mode()) {
    case 0:
    case 1: {
      if (FarScale(start_time)) return true;
    } break;

    case 3:
    case 2: {
      if (ThreeCubeAuto(start_time)) return true;
    } break;
  }
  /*
  switch (params->mode()) {
    case 1: {
      if (FarScale(start_time)) return true;
      //if (CloseSwitch(start_time)) return true;
    } break;
    case 0: {
      if (DriveStraight()) return true;
    } break;
    case 200: {
      if (FarSwitch(start_time)) return true;
    } break;

    case 3:
    case 2: {
      if (ThreeCubeAuto(start_time)) {
        return true;
      }
    } break;
  }
  */

  AOS_LOG(INFO, "Done %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      event_loop()->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);

  while (!ShouldCancel()) {
    phased_loop.SleepUntilNext();
  }
  AOS_LOG(DEBUG, "Done running\n");

  return true;
}

bool AutonomousActor::FarSwitch(monotonic_clock::time_point start_time,
                                bool drive_behind, bool left) {
  const double turn_scalar = left ? 1.0 : -1.0;
  if (drive_behind) {
    // Start on the left.   Hit the switch.
    constexpr double kFullDriveLength = 9.93;
    constexpr double kTurnDistance = 4.40;
    StartDrive(-kFullDriveLength, 0.0, kDrive, kTurn);
    set_arm_goal_position(arm::NeutralIndex());
    set_grab_box(false);
    SendSuperstructureGoal();
    if (!WaitForDriveProfileNear(kFullDriveLength - (kTurnDistance - 1.4)))
      return true;
    StartDrive(0.0, 0.0, kFarSwitchTurnDrive, kTurn);

    if (!WaitForDriveProfileNear(kFullDriveLength - kTurnDistance)) return true;
    StartDrive(0.0, turn_scalar * (-M_PI / 2.0), kFarSwitchTurnDrive,
               kSweepingTurn);
    if (!WaitForTurnProfileDone()) return true;

    // Now, close so let's move the arm up.
    set_arm_goal_position(arm::FrontSwitchAutoIndex());
    SendSuperstructureGoal();

    StartDrive(0.0, 0.0, kDrive, kTurn);
    if (!WaitForDriveProfileNear(2.0)) return true;
    StartDrive(0.0, 0.0, kFarSwitchTurnDrive, kFastTurn);

    if (!WaitForDriveProfileNear(1.3)) return true;

    StartDrive(0.0, turn_scalar * M_PI / 2.0, kFarSwitchTurnDrive, kFastTurn);
    if (!WaitForTurnProfileDone()) return true;

    constexpr double kGentlePushDrive = 0.70;
    StartDrive(kGentlePushDrive, 0.0, kSlowDrive, kTurn);

    if (!WaitForDriveProfileNear(kGentlePushDrive - 0.25)) return true;
    // Turn down the peak voltage when we push against the wall so we don't blow
    // breakers or pull the battery voltage down too far.
    set_max_drivetrain_voltage(6.0);
    StartDrive(0.00, 0.0, kFinalSwitchDrive, kTurn);

    if (!WaitForArmTrajectoryClose(0.001)) return true;
    AOS_LOG(INFO, "Arm close at %f\n",
            ::aos::time::DurationInSeconds(monotonic_now() - start_time));

    ::std::this_thread::sleep_for(chrono::milliseconds(1000));

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

    if (!WaitForDriveProfileNear(kFullDriveLength - kTurnDistance)) return true;
    StartDrive(0.0, turn_scalar * (-M_PI / 2.0), kFarSwitchTurnDrive,
               kSweepingTurn);
    if (!WaitForTurnProfileDone()) return true;
    StartDrive(0.0, 0.0, kDrive, kTurn);
    if (!WaitForDriveProfileDone()) return true;

    // Now, close so let's move the arm up.
    set_arm_goal_position(arm::FrontSwitchIndex());
    SendSuperstructureGoal();

    StartDrive(0.0, turn_scalar * (-M_PI / 2.0), kDrive, kFastTurn);
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
  return false;
}

bool AutonomousActor::FarScale(monotonic_clock::time_point start_time) {
  // Start on the left.   Hit the switch.
  constexpr double kFullDriveLength = 11.40;
  constexpr double kFirstTurnDistance = 4.40;
  constexpr double kSecondTurnDistance = 9.30;
  StartDrive(-kFullDriveLength, 0.0, kDrive, kTurn);
  if (!WaitForDriveProfileNear(kFullDriveLength - (kFirstTurnDistance - 1.4)))
    return true;
  StartDrive(0.0, 0.0, kFarSwitchTurnDrive, kTurn);

  if (!WaitForDriveProfileNear(kFullDriveLength - kFirstTurnDistance))
    return true;
  StartDrive(0.0, -M_PI / 2.0, kFarSwitchTurnDrive, kSweepingTurn);
  set_arm_goal_position(arm::BackHighBoxIndex());
  SendSuperstructureGoal();
  if (!WaitForTurnProfileDone()) return true;

  StartDrive(0.0, 0.0, kDrive, kTurn);

  if (!WaitForDriveProfileNear(kFullDriveLength - (kSecondTurnDistance - 1.4)))
    return true;

  StartDrive(0.0, 0.0, kFarScaleFinalTurnDrive, kTurn);
  if (!WaitForDriveProfileNear(kFullDriveLength - (kSecondTurnDistance)))
    return true;
  AOS_LOG(INFO, "Final turn at %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));

  StartDrive(0.0, M_PI / 2.0, kFarScaleFinalTurnDrive, kFarScaleSweepingTurn);
  if (!WaitForDriveProfileNear(0.15)) return true;

  StartDrive(0.0, 0.3, kFarScaleFinalTurnDrive, kFarScaleSweepingTurn);

  AOS_LOG(INFO, "Dropping at %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  set_open_claw(true);
  SendSuperstructureGoal();

  ::std::this_thread::sleep_for(chrono::milliseconds(1000));
  AOS_LOG(INFO, "Backing up at %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));

  StartDrive(1.5, -0.55, kFarScaleFinalTurnDrive, kFarScaleSweepingTurn);

  if (!WaitForDriveProfileNear(1.4)) return true;
  set_arm_goal_position(arm::NeutralIndex());
  set_open_claw(false);
  set_grab_box(true);
  SendSuperstructureGoal();

  if (!WaitForDriveProfileNear(0.3)) return true;

  set_intake_angle(0.15);
  set_roller_voltage(10.0);
  SendSuperstructureGoal();

  set_max_drivetrain_voltage(6.0);
  StartDrive(0.0, 0.0, kFarScaleFinalTurnDrive, kFarScaleSweepingTurn);

  if (!WaitForDriveProfileDone()) return true;
  if (!WaitForTurnProfileDone()) return true;

  StartDrive(0.2, 0.0, kThirdBoxSlowBackup, kThirdBoxSlowTurn);

  if (!WaitForBoxGrabed()) return true;
  set_arm_goal_position(arm::BackHighBoxIndex());
  set_intake_angle(-3.0);
  set_roller_voltage(0.0);
  set_grab_box(false);
  SendSuperstructureGoal();

  StartDrive(-1.40, 0.15, kThirdBoxPlaceDrive, kFarScaleSweepingTurn);

  if (!WaitForDriveProfileNear(0.1)) return true;

  StartDrive(0.0, 0.5, kThirdBoxPlaceDrive, kFarScaleSweepingTurn);

  if (!WaitForDriveProfileDone()) return true;
  if (!WaitForTurnProfileDone()) return true;
  ::std::this_thread::sleep_for(chrono::milliseconds(500));

  AOS_LOG(INFO, "Dropping second box at %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  set_open_claw(true);
  SendSuperstructureGoal();

  ::std::this_thread::sleep_for(chrono::milliseconds(1000));

  StartDrive(1.4, -0.7, kThirdBoxPlaceDrive, kFarScaleSweepingTurn);
  ::std::this_thread::sleep_for(chrono::milliseconds(200));
  set_arm_goal_position(arm::NeutralIndex());
  set_open_claw(false);
  SendSuperstructureGoal();

  if (!WaitForDriveProfileNear(1.0)) return true;

  StartDrive(0.0, -0.6, kThirdBoxPlaceDrive, kFarScaleSweepingTurn);
  return false;
}

bool AutonomousActor::FarReadyScale(monotonic_clock::time_point start_time) {
  // Start on the left.   Hit the switch.
  constexpr double kFullDriveLength = 7.5;
  constexpr double kFirstTurnDistance = 4.40;
  StartDrive(-kFullDriveLength, 0.0, kDrive, kTurn);
  if (!WaitForDriveProfileNear(kFullDriveLength - (kFirstTurnDistance - 1.4)))
    return true;
  StartDrive(0.0, 0.0, kFarSwitchTurnDrive, kTurn);

  if (!WaitForDriveProfileNear(kFullDriveLength - kFirstTurnDistance))
    return true;
  StartDrive(0.0, -M_PI / 2.0, kFarSwitchTurnDrive, kSweepingTurn);
  set_arm_goal_position(arm::UpIndex());
  SendSuperstructureGoal();
  AOS_LOG(INFO, "Lifting arm at %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  if (!WaitForTurnProfileDone()) return true;

  StartDrive(0.0, 0.0, kDrive, kTurn);
  return false;
}

bool AutonomousActor::DriveStraight() {
  StartDrive(-3.2, 0.0, kDrive, kTurn);
  if (!WaitForDriveProfileDone()) return true;
  return false;
}

bool AutonomousActor::CloseSwitch(monotonic_clock::time_point start_time,
                                  bool left) {
  const double turn_scalar = left ? 1.0 : -1.0;

  constexpr double kDriveDistance = 3.2;
  // Start on the left.   Drive, arc a turn, and drop in the close switch.
  StartDrive(-kDriveDistance, 0.0, kDrive, kTurn);
  if (!WaitForDriveNear(2.5, M_PI / 2.0)) return true;

  // Now, close so let's move the arm up.
  set_arm_goal_position(arm::BackSwitchIndex());
  SendSuperstructureGoal();

  StartDrive(0.0, 0.0, kSlowDrive, kSweepingTurn);
  if (!WaitForDriveNear(1.6, M_PI / 2.0)) return true;

  StartDrive(0.0, turn_scalar * (-M_PI / 4.0 - 0.2), kSlowDrive, kSweepingTurn);
  if (!WaitForDriveNear(0.2, 0.2)) return true;
  set_max_drivetrain_voltage(6.0);
  AOS_LOG(INFO, "Lowered drivetrain voltage %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  ::std::this_thread::sleep_for(chrono::milliseconds(300));

  set_open_claw(true);
  SendSuperstructureGoal();

  ::std::this_thread::sleep_for(chrono::milliseconds(500));
  set_max_drivetrain_voltage(12.0);
  StartDrive(0.7, 0.0, kDrive, kTurn);
  if (!WaitForTurnProfileDone()) return true;

  set_arm_goal_position(arm::NeutralIndex());
  SendSuperstructureGoal();
  return false;
}

bool AutonomousActor::ThreeCubeAuto(monotonic_clock::time_point start_time) {
  // Start on the left.   Hit the scale.
  constexpr double kDriveDistance = 6.95;
  // Distance and angle to do the big drive to the third cube.
  constexpr double kThirdCubeDrive = 1.57 + 0.05 + 0.05;
  constexpr double kThirdCubeTurn = M_PI / 4.0 + 0.1;
  // Angle to do the slow pickup turn on the third box.
  constexpr double kThirdBoxEndTurnAngle = 0.30;

  // Distance to drive back to the scale with the third cube.
  constexpr double kThirdCubeDropDrive = kThirdCubeDrive + 0.40;

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
  set_intake_angle(-0.60);
  AOS_LOG(INFO, "Dropped first box %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));

  ::std::this_thread::sleep_for(chrono::milliseconds(700));

  set_grab_box(true);
  SendSuperstructureGoal();

  AOS_LOG(INFO, "Starting second box drive %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  constexpr double kSecondBoxSwerveAngle = 0.35;
  constexpr double kSecondBoxDrive = 1.38;
  StartDrive(kSecondBoxDrive, 0.0, kDrive, kFastTurn);
  if (!WaitForDriveNear(kSecondBoxDrive - 0.2, M_PI / 2.0)) return true;

  StartDrive(0.0, kSecondBoxSwerveAngle, kDrive, kFastTurn);
  if (!WaitForDriveNear(0.5, M_PI / 2.0)) return true;

  set_open_claw(true);
  set_disable_box_correct(false);
  SendSuperstructureGoal();

  StartDrive(0.0, -kSecondBoxSwerveAngle, kDrive, kFastTurn);

  if (!WaitForDriveProfileDone()) return true;
  set_max_drivetrain_voltage(6.0);
  StartDrive(0.0, 0.0, kDrive, kFastTurn);

  set_intake_angle(0.15);
  set_arm_goal_position(arm::BackHighBoxIndex());
  set_open_claw(false);

  set_roller_voltage(10.0);
  SendSuperstructureGoal();

  AOS_LOG(INFO, "Grabbing second box %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  ::std::this_thread::sleep_for(chrono::milliseconds(200));
  StartDrive(-0.04, 0.0, kThirdBoxSlowBackup, kThirdBoxSlowTurn);

  if (!WaitForBoxGrabed()) return true;
  set_max_drivetrain_voltage(12.0);

  AOS_LOG(INFO, "Got second box %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  ::std::this_thread::sleep_for(chrono::milliseconds(500));

  set_grab_box(false);
  // set_arm_goal_position(arm::UpIndex());
  set_arm_goal_position(arm::BackHighBoxIndex());
  set_roller_voltage(0.0);
  set_disable_box_correct(false);
  SendSuperstructureGoal();
  AOS_LOG(INFO, "Driving to place second box %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));

  StartDrive(-kSecondBoxDrive + 0.16, kSecondBoxSwerveAngle, kDrive, kFastTurn);
  if (!WaitForDriveNear(0.4, M_PI / 2.0)) return true;

  constexpr double kSecondBoxEndExtraAngle = 0.3;
  StartDrive(0.0, -kSecondBoxSwerveAngle - kSecondBoxEndExtraAngle, kDrive,
             kFastTurn);

  AOS_LOG(INFO, "Starting throw %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  if (!WaitForDriveNear(0.4, M_PI / 2.0)) return true;
  if (!WaitForArmTrajectoryClose(0.25)) return true;
  SendSuperstructureGoal();

  // Throw the box.
  // if (!WaitForArmTrajectoryClose(0.20)) return true;
  if (!WaitForDriveNear(0.2, M_PI / 2.0)) return true;

  set_open_claw(true);
  set_intake_angle(-M_PI / 4.0);
  AOS_LOG(INFO, "Releasing second box %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  SendSuperstructureGoal();

  ::std::this_thread::sleep_for(chrono::milliseconds(700));
  set_open_claw(false);
  SendSuperstructureGoal();

  AOS_LOG(INFO, "Driving to third box %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  StartDrive(kThirdCubeDrive, kSecondBoxEndExtraAngle, kThirdBoxDrive,
             kFastTurn);
  if (!WaitForDriveNear(kThirdCubeDrive - 0.1, M_PI / 4.0)) return true;
  set_grab_box(true);
  SendSuperstructureGoal();

  StartDrive(0.0, kThirdCubeTurn, kThirdBoxDrive, kFastTurn);
  if (!WaitForDriveNear(0.05, M_PI / 4.0)) return true;

  set_intake_angle(0.05);
  set_roller_voltage(9.0);
  SendSuperstructureGoal();

  if (!WaitForDriveProfileDone()) return true;
  if (!WaitForTurnProfileDone()) return true;
  StartDrive(0.35, kThirdBoxEndTurnAngle, kThirdBoxSlowBackup,
             kThirdBoxSlowTurn);
  if (!WaitForDriveProfileDone()) return true;

  AOS_LOG(INFO, "Waiting for third box %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  if (!WaitForBoxGrabed()) return true;
  AOS_LOG(INFO, "Third box grabbed %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  const bool too_late =
      monotonic_now() > start_time + chrono::milliseconds(12500);
  if (too_late) {
    AOS_LOG(INFO, "Third box too long, going up. %f\n",
            ::aos::time::DurationInSeconds(monotonic_now() - start_time));
    set_grab_box(false);
    set_arm_goal_position(arm::UpIndex());
    set_roller_voltage(0.0);
    SendSuperstructureGoal();
  }
  ::std::this_thread::sleep_for(chrono::milliseconds(400));

  set_grab_box(false);
  if (!too_late) {
    set_arm_goal_position(arm::BackMiddle2BoxIndex());
  }
  set_roller_voltage(0.0);
  SendSuperstructureGoal();

  StartDrive(-kThirdCubeDropDrive, 0.0, kThirdBoxPlaceDrive, kReallyFastTurn);

  if (!WaitForDriveNear(kThirdCubeDropDrive - 0.23, M_PI / 4.0)) return true;
  StartDrive(0.0, -kThirdCubeTurn - 0.2 - kThirdBoxEndTurnAngle - 0.3,
             kThirdBoxPlaceDrive, kReallyFastTurn);

  if (!WaitForDriveNear(0.30, M_PI / 4.0 + 0.2)) return true;

  if (!too_late) {
    set_open_claw(true);
    set_intake_angle(-M_PI / 4.0);
    set_roller_voltage(0.0);
    SendSuperstructureGoal();

    AOS_LOG(INFO, "Final open %f\n",
            ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  }

  if (!WaitForDriveProfileDone()) return true;
  if (!WaitForTurnProfileDone()) return true;

  ::std::this_thread::sleep_for(chrono::milliseconds(14750) -
                                (monotonic_now() - start_time));

  set_arm_goal_position(arm::UpIndex());
  SendSuperstructureGoal();

  ::std::this_thread::sleep_for(chrono::milliseconds(15000) -
                                (monotonic_now() - start_time));

  set_close_claw(true);
  SendSuperstructureGoal();
  return false;
}

}  // namespace actors
}  // namespace y2018
