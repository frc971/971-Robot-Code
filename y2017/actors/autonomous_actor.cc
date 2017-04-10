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

const ProfileParameters kSlowDrive = {3.0, 2.0};
const ProfileParameters kSlowTurn = {3.0, 3.0};
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
    case 0:
    default: {
      constexpr double kDriveDirection = 1.0;
      // Test case autonomous mode.
      // Drives forward 1.0 meters and then turns 180 degrees.
      set_intake_goal(0.07);
      SendSuperstructureGoal();
      StartDrive(-3.7, 0.0, kSlowDrive, kSlowTurn);
      if (!WaitForDriveNear(2.75, 0.0)) return true;

      set_intake_goal(0.23);
      set_turret_goal(0.0);
      // Values good for blue:
      // TODO(austin): Drive these off the auto switch.
      //set_hood_goal(0.355 + 0.01 + 0.005);
      //set_shooter_velocity(355.0);
      set_hood_goal(0.355 + 0.01 + 0.005 + 0.01 + 0.01);
      set_shooter_velocity(351.0);
      SendSuperstructureGoal();
      StartDrive(0.0, -M_PI / 4.0 * kDriveDirection, kSlowDrive, kSlowTurn);
      if (!WaitForDriveNear(0.05, 0.0)) return true;

      this_thread::sleep_for(chrono::milliseconds(100));

      StartDrive(0.0, (M_PI / 4.0 + 0.20) * kDriveDirection, kSlowDrive,
                 kSmashTurn);
      if (!WaitForDriveNear(0.05, 0.2)) return true;

      set_vision_track(true);

      set_indexer_angular_velocity(-1.8 * M_PI);
      SendSuperstructureGoal();

      this_thread::sleep_for(chrono::milliseconds(200));

      StartDrive(0.0, (-0.15) * kDriveDirection, kSlowDrive, kSlowTurn);
      if (!WaitForDriveNear(0.05, 0.02)) return true;

      LOG(INFO, "Started shooting at %f\n",
          DoubleSeconds(monotonic_clock::now() - start_time));

      this_thread::sleep_for(start_time + chrono::seconds(9) -
                             monotonic_clock::now());
      StartDrive(0.0, (-0.05) * kDriveDirection, kSlowDrive, kSlowTurn);
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
