#include "y2017/actors/autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2017/control_loops/drivetrain/drivetrain_base.h"

namespace y2017 {
namespace actors {
using ::frc971::control_loops::drivetrain_queue;
using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;

namespace {

double DoubleSeconds(monotonic_clock::duration duration) {
  return ::std::chrono::duration_cast<::std::chrono::duration<double>>(duration)
      .count();
}

const ProfileParameters kGearBallBackDrive = {3.0, 3.5};
const ProfileParameters kGearDrive = {1.5, 2.0};
const ProfileParameters kGearFastDrive = {2.0, 2.5};
const ProfileParameters kGearSlowDrive = {1.0, 2.0};
const ProfileParameters kGearPlaceDrive = {0.40, 2.0};
const ProfileParameters kSlowDrive = {3.0, 2.0};
const ProfileParameters kSlowTurn = {3.0, 3.0};
const ProfileParameters kFirstTurn = {1.0, 1.5};
const ProfileParameters kFirstGearStartTurn = {2.0, 3.0};
const ProfileParameters kFirstGearTurn = {2.0, 5.0};
const ProfileParameters kSecondGearTurn = {3.0, 5.0};
const ProfileParameters kSmashTurn = {3.0, 5.0};

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
    default: {
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
      LOG(INFO, "Turn Middle: %f left to go\n", DriveDistanceLeft());

      StartDrive(0.0, 0.0, kGearDrive, kFirstGearTurn);

      if (!WaitForTurnProfileDone()) return true;
      LOG(INFO, "Turn profile ended: %f left to go\n", DriveDistanceLeft());

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
      //set_indexer_angular_velocity(-2.1 * M_PI);
      //SendSuperstructureGoal();
      //this_thread::sleep_for(chrono::milliseconds(1750));

      //this_thread::sleep_for(chrono::milliseconds(500));
      this_thread::sleep_for(chrono::milliseconds(750));
      set_indexer_angular_velocity(0.0);
      set_vision_track(false);
      set_turret_goal(0.0);

      set_hood_goal(0.37);
      set_shooter_velocity(351.0);
      set_intake_goal(0.18);
      set_gear_servo(0.4);

      SendSuperstructureGoal();
      LOG(INFO, "Starting drive back %f\n",
          DoubleSeconds(monotonic_clock::now() - start_time));

      StartDrive(-2.69, kDriveDirection * 1.30, kSlowDrive,
                 kFirstGearStartTurn);
      if (!WaitForDriveNear(2.4, 0.0)) return true;

      if (!WaitForTurnProfileDone()) return true;
      StartDrive(0.0, 0.0, kGearBallBackDrive, kFirstGearStartTurn);

      if (!WaitForDriveNear(0.2, 0.0)) return true;
      this_thread::sleep_for(chrono::milliseconds(200));
      StartDrive(0.0, kDriveDirection * 1.10, kSlowDrive,
                 kSmashTurn);

      if (!WaitForDriveNear(0.2, 0.35)) return true;
      set_vision_track(true);
      set_use_vision_for_shots(true);
      SendSuperstructureGoal();

      if (!WaitForDriveNear(0.2, 0.2)) return true;
      StartDrive(0.0, -kDriveDirection * 0.15, kSlowDrive,
                 kSmashTurn);

      LOG(INFO, "Starting second shot %f\n",
          DoubleSeconds(monotonic_clock::now() - start_time));
      set_indexer_angular_velocity(-2.15 * M_PI);
      SendSuperstructureGoal();
      if (!WaitForDriveNear(0.2, 0.1)) return true;

      this_thread::sleep_for(start_time + chrono::seconds(11) -
                             monotonic_clock::now());
      if (ShouldCancel()) return true;
      set_intake_max_velocity(0.05);
      set_intake_goal(0.08);

      this_thread::sleep_for(start_time + chrono::seconds(15) -
                             monotonic_clock::now());
      if (ShouldCancel()) return true;

      set_intake_max_velocity(0.50);
      set_intake_goal(0.18);
      set_shooter_velocity(0.0);
      set_indexer_angular_velocity(0.0);
      SendSuperstructureGoal();

    } break;

    case 500: {
      // Red is positive.
      constexpr double kDriveDirection = 1.0;
      // Side hopper auto.
      set_intake_goal(0.07);
      SendSuperstructureGoal();

      StartDrive(-3.3, kDriveDirection * M_PI / 10, kSlowDrive, kFirstTurn);
      if (!WaitForDriveNear(3.30, 0.0)) return true;
      LOG(INFO, "Turn ended: %f left to go\n", DriveDistanceLeft());
      // We can go to 2.50 before we hit the previous profile.

      if (!WaitForDriveNear(2.42, 0.0)) return true;
      LOG(INFO, "%f left to go\n", DriveDistanceLeft());

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

      StartDrive(0.0, (M_PI / 8.0 + 0.20) * kDriveDirection, kSlowDrive,
                 kSmashTurn);
      if (!WaitForDriveNear(0.05, 0.2)) return true;

      set_vision_track(true);
      set_use_vision_for_shots(true);

      set_indexer_angular_velocity(-2.1 * M_PI);
      SendSuperstructureGoal();

      this_thread::sleep_for(chrono::milliseconds(200));

      StartDrive(0.0, (-0.15) * kDriveDirection, kSlowDrive, kSlowTurn);
      if (!WaitForDriveNear(0.05, 0.02)) return true;

      LOG(INFO, "Started shooting at %f\n",
          DoubleSeconds(monotonic_clock::now() - start_time));

      this_thread::sleep_for(start_time + chrono::seconds(9) -
                             monotonic_clock::now());
      if (ShouldCancel()) return true;

      set_intake_max_velocity(0.05);
      set_intake_goal(0.08);
      SendSuperstructureGoal();

      this_thread::sleep_for(start_time + chrono::seconds(15) -
                             monotonic_clock::now());
      if (ShouldCancel()) return true;

      set_shooter_velocity(0.0);
      set_indexer_angular_velocity(0.0);
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
}  // namespace y2017
