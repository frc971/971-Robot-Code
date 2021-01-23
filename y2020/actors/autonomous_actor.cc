#include "y2020/actors/autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/logging/logging.h"
#include "aos/util/math.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"
#include "y2020/actors/auto_splines.h"

DEFINE_bool(spline_auto, false, "If true, define a spline autonomous mode");

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
          "/drivetrain")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")) {
  set_max_drivetrain_voltage(2.0);
}

void AutonomousActor::Reset() {
  InitializeEncoders();
  ResetDrivetrain();

  joystick_state_fetcher_.Fetch();
  CHECK(joystick_state_fetcher_.get() != nullptr)
      << "Expect at least one JoystickState message before running auto...";
  alliance_ = joystick_state_fetcher_->alliance();
  // Set up the starting position for the blue alliance.
  // Currently just arbitrarily chosen for testing purposes, such that the
  // robot starts on the side of the field nearest where it would score
  // (although I do not know if this is actually a legal starting position).
  Eigen::Vector2d starting_position(3.3, -0.7);
  double starting_heading = 0.0;
  if (alliance_ == aos::Alliance::kRed) {
    starting_position *= -1.0;
    starting_heading = aos::math::NormalizeAngle(starting_heading + M_PI);
  }
  if (FLAGS_spline_auto) {
    // TODO(james): Resetting the localizer breaks the left/right statespace
    // controller.  That is a bug, but we can fix that later by not resetting.
    auto builder = localizer_control_sender_.MakeBuilder();

    LocalizerControl::Builder localizer_control_builder =
        builder.MakeBuilder<LocalizerControl>();
    localizer_control_builder.add_x(starting_position.x());
    localizer_control_builder.add_y(starting_position.y());
    localizer_control_builder.add_theta(starting_heading);
    localizer_control_builder.add_theta_uncertainty(0.00001);
    if (!builder.Send(localizer_control_builder.Finish())) {
      AOS_LOG(ERROR, "Failed to reset localizer.\n");
    }
  }
}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  Reset();
  AOS_LOG(INFO, "Params are %d\n", params->mode());
  if (alliance_ == aos::Alliance::kInvalid) {
    AOS_LOG(INFO, "Aborting autonomous due to invalid alliance selection.");
    return false;
  }
  if (FLAGS_spline_auto) {
    SplineAuto();
  } else {
    return DriveFwd();
  }
  return true;
}

void AutonomousActor::SplineAuto() {
  SplineHandle spline1 = PlanSpline(std::bind(AutonomousSplines::BasicSSpline,
                                              std::placeholders::_1, alliance_),
                                    SplineDirection::kForward);

  if (!spline1.WaitForPlan()) return;
  spline1.Start();

  if (!spline1.WaitForSplineDistanceRemaining(0.02)) return;
}

ProfileParametersT MakeProfileParametersT(const float max_velocity,
                                          const float max_acceleration) {
  ProfileParametersT params;
  params.max_velocity = max_velocity;
  params.max_acceleration = max_acceleration;
  return params;
}

bool AutonomousActor::DriveFwd() {
  const ProfileParametersT kDrive = MakeProfileParametersT(0.1f, 0.5f);
  const ProfileParametersT kTurn = MakeProfileParametersT(5.0f, 15.0f);
  StartDrive(0.5, 0.0, kDrive, kTurn);
  return WaitForDriveDone();
}
}  // namespace actors
}  // namespace y2020
