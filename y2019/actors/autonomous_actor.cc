#include "y2019/actors/autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/logging/logging.h"
#include "aos/util/phased_loop.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/localizer.q.h"
#include "y2019/actors/auto_splines.h"
#include "y2019/control_loops/drivetrain/drivetrain_base.h"

namespace y2019 {
namespace actors {
using ::frc971::control_loops::drivetrain_queue;
using ::frc971::control_loops::drivetrain::localizer_control;
using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;

namespace {

double DoubleSeconds(monotonic_clock::duration duration) {
  return ::std::chrono::duration_cast<::std::chrono::duration<double>>(duration)
      .count();
}

}  // namespace

AutonomousActor::AutonomousActor(
    ::frc971::autonomous::AutonomousActionQueueGroup *s)
    : frc971::autonomous::BaseAutonomousActor(
          s, control_loops::drivetrain::GetDrivetrainConfig()) {}

void AutonomousActor::Reset(bool is_left) {
  const double turn_scalar = is_left ? 1.0 : -1.0;
  elevator_goal_ = 0.01;
  wrist_goal_ = -M_PI / 2.0;
  intake_goal_ = -1.2;

  suction_on_ = false;
  suction_gamepiece_ = 1;

  elevator_max_velocity_ = 0.0;
  elevator_max_acceleration_ = 0.0;
  wrist_max_velocity_ = 0.0;
  wrist_max_acceleration_ = 0.0;

  InitializeEncoders();
  SendSuperstructureGoal();

  {
    auto localizer_resetter = localizer_control.MakeMessage();
    // Start on the left l2.
    localizer_resetter->x = 1.0;
    localizer_resetter->y = 1.5 * turn_scalar;
    localizer_resetter->theta = M_PI;
    localizer_resetter->theta_uncertainty = 0.0000001;
    if (!localizer_resetter.Send()) {
      LOG(ERROR, "Failed to reset localizer.\n");
    }
  }

  // Wait for the drivetrain to run so it has time to reset the heading.
  // Otherwise our drivetrain reset will do a 180 right at the start.
  drivetrain_queue.status.FetchAnother();
  LOG(INFO, "Heading is %f\n", drivetrain_queue.status->estimated_heading);
  InitializeEncoders();
  ResetDrivetrain();
  drivetrain_queue.status.FetchAnother();
  LOG(INFO, "Heading is %f\n", drivetrain_queue.status->estimated_heading);

  ResetDrivetrain();
  InitializeEncoders();
}

const ProfileParameters kJumpDrive = {2.0, 3.0};
const ProfileParameters kDrive = {4.0, 3.0};
const ProfileParameters kTurn = {5.0, 15.0};

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams &params) {
  monotonic_clock::time_point start_time = monotonic_clock::now();
  const bool is_left = params.mode == 0;

  {
    LOG(INFO, "Starting autonomous action with mode %" PRId32 " %s\n",
        params.mode, is_left ? "left" : "right");
  }
  const double turn_scalar = is_left ? 1.0 : -1.0;

  Reset(is_left);

  // Grab the disk, wait until we have vacuum, then jump
  set_elevator_goal(0.01);
  set_wrist_goal(-M_PI / 2.0);
  set_intake_goal(-1.2);
  set_suction_goal(true, 1);
  SendSuperstructureGoal();

  if (!WaitForGamePiece()) return true;
  LOG(INFO, "Has game piece\n");

  StartDrive(-4.0, 0.0, kJumpDrive, kTurn);
  if (!WaitForDriveNear(3.3, 10.0)) return true;
  LOG(INFO, "Lifting\n");
  set_elevator_goal(0.30);
  SendSuperstructureGoal();

  if (!WaitForDriveNear(2.8, 10.0)) return true;
  LOG(INFO, "Off the platform\n");

  StartDrive(0.0, 1.00 * turn_scalar, kDrive, kTurn);
  LOG(INFO, "Turn started\n");
  if (!WaitForSuperstructureDone()) return true;
  LOG(INFO, "Superstructure done\n");

  if (!WaitForDriveNear(0.7, 10.0)) return true;
  StartDrive(0.0, -0.35 * turn_scalar, kDrive, kTurn);

  LOG(INFO, "Elevator up\n");
  set_elevator_goal(0.78);
  SendSuperstructureGoal();

  if (!WaitForDriveDone()) return true;
  LOG(INFO, "Done driving\n");

  if (!WaitForSuperstructureDone()) return true;

  LOG(INFO, "Done %f\n", DoubleSeconds(monotonic_clock::now() - start_time));

  return true;
}

}  // namespace actors
}  // namespace y2019
