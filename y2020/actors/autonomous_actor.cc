#include "y2020/actors/autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/logging/logging.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"
#include "y2020/actors/auto_splines.h"

namespace y2020 {
namespace actors {

using ::aos::monotonic_clock;
using ::frc971::ProfileParametersT;
using frc971::control_loops::drivetrain::LocalizerControl;
namespace chrono = ::std::chrono;

AutonomousActor::AutonomousActor(::aos::EventLoop *event_loop)
    : frc971::autonomous::BaseAutonomousActor(
          event_loop, control_loops::drivetrain::GetDrivetrainConfig()),
      localizer_control_sender_(event_loop->MakeSender<
          ::frc971::control_loops::drivetrain::LocalizerControl>(
          "/drivetrain")) {}

void AutonomousActor::Reset() {
  InitializeEncoders();
  ResetDrivetrain();

  {
    auto builder = localizer_control_sender_.MakeBuilder();

    LocalizerControl::Builder localizer_control_builder =
        builder.MakeBuilder<LocalizerControl>();
    // TODO(james): Set starting position based on the alliance.
    localizer_control_builder.add_x(0.0);
    localizer_control_builder.add_y(0.0);
    localizer_control_builder.add_theta(0.0);
    localizer_control_builder.add_theta_uncertainty(0.00001);
    if (!builder.Send(localizer_control_builder.Finish())) {
      AOS_LOG(ERROR, "Failed to reset localizer.\n");
    }
  }
}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  Reset();

  SplineHandle spline1 =
      PlanSpline(AutonomousSplines::BasicSSpline, SplineDirection::kForward);

  if (!spline1.WaitForPlan()) return true;
  spline1.Start();

  if (!spline1.WaitForSplineDistanceRemaining(0.02)) return true;

  AOS_LOG(INFO, "Params are %d\n", params->mode());
  return true;
}

}  // namespace actors
}  // namespace y2020
