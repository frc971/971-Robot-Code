#include "y2017/actors/autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/logging/logging.h"
#include "aos/util/phased_loop.h"
#include "y2017/control_loops/drivetrain/drivetrain_base.h"

namespace y2017 {
namespace actors {
using ::aos::monotonic_clock;
using ::frc971::ProfileParametersT;
namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;

namespace {

ProfileParametersT MakeProfileParameters(float max_velocity,
                                         float max_acceleration) {
  ProfileParametersT result;
  result.max_velocity = max_velocity;
  result.max_acceleration = max_acceleration;
  return result;
}

const ProfileParametersT kGearBallBackDrive = MakeProfileParameters(3.0, 3.5);
const ProfileParametersT kGearDrive = MakeProfileParameters(1.5, 2.0);
const ProfileParametersT kGearFastDrive = MakeProfileParameters(2.0, 2.5);
const ProfileParametersT kGearSlowDrive = MakeProfileParameters(1.0, 2.0);
const ProfileParametersT kGearPlaceDrive = MakeProfileParameters(0.40, 2.0);
const ProfileParametersT kSlowDrive = MakeProfileParameters(3.0, 2.0);
const ProfileParametersT kSlowTurn = MakeProfileParameters(3.0, 3.0);
const ProfileParametersT kFirstTurn = MakeProfileParameters(1.0, 1.5);
const ProfileParametersT kFirstGearStartTurn = MakeProfileParameters(2.0, 3.0);
const ProfileParametersT kFirstGearTurn = MakeProfileParameters(2.0, 5.0);
const ProfileParametersT kSecondGearTurn = MakeProfileParameters(3.0, 5.0);
const ProfileParametersT kSmashTurn = MakeProfileParameters(1.5, 5.0);

}  // namespace

AutonomousActor::AutonomousActor(::aos::EventLoop *event_loop)
    : frc971::autonomous::BaseAutonomousActor(
          event_loop, control_loops::drivetrain::GetDrivetrainConfig()),
      superstructure_status_fetcher_(
          event_loop
              ->MakeFetcher<::y2017::control_loops::superstructure::Status>(
                  "/superstructure")),
      superstructure_goal_sender_(
          event_loop->MakeSender<::y2017::control_loops::superstructure::Goal>(
              "/superstructure")) {}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  const monotonic_clock::time_point start_time = monotonic_now();
  AOS_LOG(INFO, "Starting autonomous action with mode %" PRId32 "\n",
          params->mode());
  Reset();

  switch (params->mode()) {
    case 503: {
      // Middle gear auto.
      // Red is positive.
      // Line up on boiler side edge.
      constexpr double kDriveDirection = 1.0;

      set_intake_goal(0.18);
      set_turret_goal(0.0);
      SendSuperstructureGoal();

      set_turret_goal(-M_PI / 4.0 * kDriveDirection);
      SendSuperstructureGoal();

      // Turn towards the peg.
      StartDrive(0.0, kDriveDirection * 0.162, kGearDrive, kSlowTurn);
      if (!WaitForDriveNear(100.0, 0.02)) return true;
      if (!WaitForTurnProfileDone()) return true;

      // Drive, but get within 0.3 meters
      StartDrive(1.73, 0.0, kGearFastDrive, kSlowTurn);
      if (!WaitForDriveNear(0.3, 0.0)) return true;

      // Now, add a slow, short move to actually place the gear.
      StartDrive(0.18, 0.0, kGearPlaceDrive, kSecondGearTurn);
      if (!WaitForDriveNear(0.07, 0.0)) return true;

      set_gear_servo(0.3);
      SendSuperstructureGoal();

      // Slow down and then pause to let Chris pull the gear off.
      this_thread::sleep_for(chrono::milliseconds(1000));

      // Back up
      StartDrive(-1.00, 0.0, kGearFastDrive, kSlowTurn);
      if (!WaitForDriveNear(0.1, 0.1)) return true;

      // Turn towards the boiler.
      StartDrive(0.0, -kDriveDirection * M_PI / 2.0, kGearFastDrive, kSlowTurn);
      if (!WaitForDriveNear(0.1, 0.1)) return true;

      // Drive up near it.
      StartDrive(1.8, 0.0, kGearFastDrive, kSlowTurn);
      if (!WaitForDriveNear(0.1, 0.1)) return true;

      set_hood_goal(0.37);
      set_intake_goal(0.23);
      set_shooter_velocity(353.0);
      set_vision_track(true);
      set_use_vision_for_shots(true);
      set_indexer_angular_velocity(-1.1 * M_PI);
      SendSuperstructureGoal();

      this_thread::sleep_for(start_time + chrono::seconds(15) -
                             monotonic_now());
      if (ShouldCancel()) return true;

      set_shooter_velocity(0.0);
      set_indexer_angular_velocity(0.0);
      SendSuperstructureGoal();

    } break;

    case 500: {
      // Red is positive.
      constexpr double kDriveDirection = -1.0;
      // Side peg + hopper auto.

      set_intake_goal(0.23);
      set_turret_goal(-M_PI * kDriveDirection);
      SendSuperstructureGoal();

      constexpr double kLongPegDrive = 3.025;

      StartDrive(kLongPegDrive, -kDriveDirection * M_PI / 4, kGearDrive,
                 kFirstGearStartTurn);
      if (!WaitForDriveNear(100.0, M_PI / 8.0)) return true;
      AOS_LOG(INFO, "Turn Middle: %f left to go\n", DriveDistanceLeft());

      StartDrive(0.0, 0.0, kGearDrive, kFirstGearTurn);

      if (!WaitForTurnProfileDone()) return true;
      AOS_LOG(INFO, "Turn profile ended: %f left to go\n", DriveDistanceLeft());

      set_hood_goal(0.43);
      set_shooter_velocity(364.0);
      SendSuperstructureGoal();

      constexpr double kTurnDistanceFromStart = 1.08;
      if (!WaitForDriveNear(kLongPegDrive - kTurnDistanceFromStart, 10.0)) {
        return true;
      }

      StartDrive(0.0, kDriveDirection * (M_PI / 4 + 0.3), kGearSlowDrive,
                 kSecondGearTurn);
      if (!WaitForTurnProfileDone()) return true;

      StartDrive(0.0, 0.0, kGearFastDrive, kSecondGearTurn);

      if (!WaitForDriveNear(0.3, 0.0)) return true;

      set_vision_track(true);
      SendSuperstructureGoal();

      StartDrive(0.19, 0.0, kGearPlaceDrive, kSecondGearTurn);

      if (!WaitForDriveNear(0.07, 0.0)) return true;
      set_gear_servo(0.3);
      SendSuperstructureGoal();

      // Shoot from the peg.
      // set_indexer_angular_velocity(-2.1 * M_PI);
      // SendSuperstructureGoal();
      // this_thread::sleep_for(chrono::milliseconds(1750));

      // this_thread::sleep_for(chrono::milliseconds(500));
      this_thread::sleep_for(chrono::milliseconds(750));
      set_indexer_angular_velocity(0.0);
      set_vision_track(false);
      set_turret_goal(0.0);

      set_hood_goal(0.37);
      set_shooter_velocity(351.0);
      set_intake_goal(0.18);
      set_gear_servo(0.4);

      SendSuperstructureGoal();
      AOS_LOG(INFO, "Starting drive back %f\n",
              ::aos::time::DurationInSeconds(monotonic_now() - start_time));

      StartDrive(-2.75, kDriveDirection * 1.24, kSlowDrive,
                 kFirstGearStartTurn);
      if (!WaitForDriveNear(2.4, 0.0)) return true;

      if (!WaitForTurnProfileDone()) return true;
      StartDrive(0.0, 0.0, kGearBallBackDrive, kFirstGearStartTurn);

      if (!WaitForDriveNear(0.2, 0.0)) return true;
      this_thread::sleep_for(chrono::milliseconds(200));
      // Trip the hopper
      StartDrive(0.0, kDriveDirection * 1.10, kSlowDrive, kSmashTurn);

      if (!WaitForDriveNear(0.2, 0.35)) return true;
      set_vision_track(true);
      set_use_vision_for_shots(true);
      SendSuperstructureGoal();

      if (!WaitForDriveNear(0.2, 0.2)) return true;
      StartDrive(0.0, -kDriveDirection * 0.15, kSlowDrive, kSmashTurn);

      AOS_LOG(INFO, "Starting second shot %f\n",
              ::aos::time::DurationInSeconds(monotonic_now() - start_time));
      set_indexer_angular_velocity(-2.15 * M_PI);
      SendSuperstructureGoal();
      if (!WaitForDriveNear(0.2, 0.1)) return true;

      this_thread::sleep_for(start_time + chrono::seconds(11) -
                             monotonic_now());
      if (ShouldCancel()) return true;
      set_intake_max_velocity(0.05);
      set_intake_goal(0.08);

      this_thread::sleep_for(start_time + chrono::seconds(15) -
                             monotonic_now());
      if (ShouldCancel()) return true;

      set_intake_max_velocity(0.50);
      set_intake_goal(0.18);
      set_shooter_velocity(0.0);
      set_indexer_angular_velocity(0.0);
      SendSuperstructureGoal();

    } break;

    default: {
      // hopper auto
      // Red is positive.
      constexpr double kDriveDirection = 1.0;
      set_intake_goal(0.07);
      SendSuperstructureGoal();

      StartDrive(-3.42, kDriveDirection * (M_PI / 10 - 0.057), kSlowDrive,
                 kFirstTurn);
      if (!WaitForDriveNear(3.30, 0.0)) return true;
      AOS_LOG(INFO, "Turn ended: %f left to go\n", DriveDistanceLeft());
      // We can go to 2.50 before we hit the previous profile.

      if (!WaitForDriveNear(2.48, 0.0)) return true;
      AOS_LOG(INFO, "%f left to go\n", DriveDistanceLeft());

      set_intake_goal(0.23);
      set_turret_goal(0.0);
      // Values good for blue:
      // TODO(austin): Drive these off the auto switch.

      set_hood_goal(0.37);
      set_shooter_velocity(353.0);
      SendSuperstructureGoal();

      StartDrive(0.0, -M_PI / 8.0 * kDriveDirection, kSlowDrive, kSlowTurn);
      if (!WaitForDriveNear(0.20, 0.0)) return true;

      this_thread::sleep_for(chrono::milliseconds(300));

      // Turn to trigger the hopper.
      StartDrive(0.0, (M_PI / 8.0 + 0.20) * kDriveDirection, kSlowDrive,
                 kSmashTurn);
      if (!WaitForDriveNear(0.05, 0.2)) return true;

      set_vision_track(true);
      set_use_vision_for_shots(true);
      SendSuperstructureGoal();

      // Now that everything is tracking, wait for the hood to zero before
      // trying to shoot.
      WaitForHoodZeroed();
      if (ShouldCancel()) return true;

      this_thread::sleep_for(chrono::milliseconds(200));

      // Turn back.
      StartDrive(0.0, (-0.15) * kDriveDirection, kSlowDrive, kSlowTurn);
      if (!WaitForDriveNear(0.05, 0.02)) return true;

      set_indexer_angular_velocity(-2.1 * M_PI);
      SendSuperstructureGoal();

      AOS_LOG(INFO, "Started shooting at %f\n",
              ::aos::time::DurationInSeconds(monotonic_now() - start_time));

      this_thread::sleep_for(start_time + chrono::seconds(9) - monotonic_now());
      if (ShouldCancel()) return true;

      set_intake_max_velocity(0.05);
      set_intake_goal(0.08);
      SendSuperstructureGoal();

      this_thread::sleep_for(start_time + chrono::seconds(15) -
                             monotonic_now());
      if (ShouldCancel()) return true;

      set_shooter_velocity(0.0);
      set_indexer_angular_velocity(0.0);
      SendSuperstructureGoal();

    } break;
  }

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

}  // namespace actors
}  // namespace y2017
