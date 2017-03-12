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

const ProfileParameters kSlowDrive = {2.0, 2.0};
const ProfileParameters kSlowTurn = {3.0, 3.0};

}  // namespace

AutonomousActor::AutonomousActor(
    ::frc971::autonomous::AutonomousActionQueueGroup *s)
    : frc971::autonomous::BaseAutonomousActor(
          s, control_loops::drivetrain::GetDrivetrainConfig()) {}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams &params) {
  monotonic_clock::time_point start_time = monotonic_clock::now();
  LOG(INFO, "Starting autonomous action with mode %" PRId32 "\n", params.mode);
  InitializeEncoders();
  ResetDrivetrain();

  switch (params.mode) {
    case 0:
    default:
      while (true) {
        constexpr auto kDelayTime = chrono::milliseconds(1);
        // Test case autonomous mode.
        // Drives forward 1.0 meters and then turns 180 degrees.
        StartDrive(1.0, 0.0, kSlowDrive, kSlowTurn);
        if (!WaitForDriveDone()) return true;

        this_thread::sleep_for(kDelayTime);
        if (ShouldCancel()) return true;

        StartDrive(0.0, M_PI, kSlowDrive, kSlowTurn);
        if (!WaitForDriveDone()) return true;

        this_thread::sleep_for(kDelayTime);
        if (ShouldCancel()) return true;

        StartDrive(1.0, 0.0, kSlowDrive, kSlowTurn);
        if (!WaitForDriveDone()) return true;

        this_thread::sleep_for(kDelayTime);
        if (ShouldCancel()) return true;

        StartDrive(0.0, M_PI, kSlowDrive, kSlowTurn);
        if (!WaitForDriveDone()) return true;

        this_thread::sleep_for(kDelayTime);
        if (ShouldCancel()) return true;
      }

      break;

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
