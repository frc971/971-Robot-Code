#include "y2020/actors/autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/logging/logging.h"
#include "aos/util/math.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "frc971/control_loops/drivetrain/spline.h"
#include "y2020/actors/auto_splines.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"

DEFINE_bool(spline_auto, true, "If true, define a spline autonomous mode");
DEFINE_bool(galactic_search, false,
            "If true, do the galactic search autonomous");
DEFINE_bool(bounce, false, "If true, run the AutoNav Bounce autonomous");
DEFINE_bool(barrel, false, "If true, run the AutoNav Barrel autonomous");
DEFINE_bool(slalom, false, "If true, run the AutoNav Slalom autonomous");

namespace y2020 {
namespace actors {

using ::aos::monotonic_clock;
using ::frc971::ProfileParametersT;
using frc971::control_loops::drivetrain::LocalizerControl;
namespace chrono = ::std::chrono;

AutonomousActor::AutonomousActor(::aos::EventLoop *event_loop)
    : frc971::autonomous::BaseAutonomousActor(
          event_loop, control_loops::drivetrain::GetDrivetrainConfig()),
      localizer_control_sender_(
          event_loop->MakeSender<
              ::frc971::control_loops::drivetrain::LocalizerControl>(
              "/drivetrain")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      path_fetcher_(event_loop->MakeFetcher<y2020::vision::GalacticSearchPath>(
          "/pi1/camera")),
      auto_splines_() {
  set_max_drivetrain_voltage(2.0);
}

void AutonomousActor::Reset() {
  InitializeEncoders();
  ResetDrivetrain();

  joystick_state_fetcher_.Fetch();
  CHECK(joystick_state_fetcher_.get() != nullptr)
      << "Expect at least one JoystickState message before running auto...";
  alliance_ = joystick_state_fetcher_->alliance();
}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  Reset();
  AOS_LOG(INFO, "Params are %d\n", params->mode());
  if (alliance_ == aos::Alliance::kInvalid) {
    AOS_LOG(INFO, "Aborting autonomous due to invalid alliance selection.");
    return false;
  }
  if (FLAGS_galactic_search) {
    GalacticSearch();
  } else if (FLAGS_bounce) {
    AutoNavBounce();
  } else if (FLAGS_barrel) {
    AutoNavBarrel();
  } else if (FLAGS_slalom) {
    AutoNavSlalom();
  } else if (FLAGS_spline_auto) {
    SplineAuto();
  } else {
    return DriveFwd();
  }
  return true;
}

void AutonomousActor::SendStartingPosition(double x, double y, double theta) {
  // Set up the starting position for the blue alliance.
  double starting_heading = theta;

  // TODO(james): Resetting the localizer breaks the left/right statespace
  // controller.  That is a bug, but we can fix that later by not resetting.
  auto builder = localizer_control_sender_.MakeBuilder();

  LocalizerControl::Builder localizer_control_builder =
      builder.MakeBuilder<LocalizerControl>();
  localizer_control_builder.add_x(x);
  localizer_control_builder.add_y(y);
  localizer_control_builder.add_theta(starting_heading);
  localizer_control_builder.add_theta_uncertainty(0.00001);
  if (!builder.Send(localizer_control_builder.Finish())) {
    AOS_LOG(ERROR, "Failed to reset localizer.\n");
  }
}

void AutonomousActor::GalacticSearch() {
  path_fetcher_.Fetch();
  CHECK(path_fetcher_.get() != nullptr)
      << "Expect at least one GalacticSearchPath message before running "
         "auto...";
  if (path_fetcher_->alliance() == y2020::vision::Alliance::kUnknown) {
    AOS_LOG(ERROR, "The galactic search path is unknown, doing nothing.");
  } else {
    SplineHandle spline1 = PlanSpline(
        [this](aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder
                   *builder) {
          flatbuffers::Offset<frc971::MultiSpline> target_spline;
          if (path_fetcher_->alliance() == y2020::vision::Alliance::kRed) {
            if (path_fetcher_->letter() == y2020::vision::Letter::kA) {
              target_spline = auto_splines_.SplineRedA(builder);
            } else {
              CHECK(path_fetcher_->letter() == y2020::vision::Letter::kB);
              target_spline = auto_splines_.SplineRedB(builder);
            }
          } else {
            if (path_fetcher_->letter() == y2020::vision::Letter::kA) {
              target_spline = auto_splines_.SplineBlueA(builder);
            } else {
              CHECK(path_fetcher_->letter() == y2020::vision::Letter::kB);
              target_spline = auto_splines_.SplineBlueB(builder);
            }
          }
          const frc971::MultiSpline *const spline =
              flatbuffers::GetTemporaryPointer(*builder->fbb(), target_spline);

          SendStartingPosition(CHECK_NOTNULL(spline));

          return target_spline;
        },
        SplineDirection::kForward);

    set_intake_goal(1.2);
    set_roller_voltage(7.0);
    SendSuperstructureGoal();

    if (!spline1.WaitForPlan()) return;
    spline1.Start();

    if (!spline1.WaitForSplineDistanceRemaining(0.02)) return;
    set_intake_goal(-0.89);
    set_roller_voltage(0.0);
    SendSuperstructureGoal();
  }
}

void AutonomousActor::AutoNavBounce() {
  SplineHandle spline1 = PlanSpline(
      [this](aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder
                 *builder) {
        flatbuffers::Offset<frc971::MultiSpline> target_spline =
            auto_splines_.AutoNavBounce1(builder);
        const frc971::MultiSpline *const spline =
            flatbuffers::GetTemporaryPointer(*builder->fbb(), target_spline);
        SendStartingPosition(CHECK_NOTNULL(spline));
        return target_spline;
      },
      SplineDirection::kForward);

  if (!spline1.WaitForPlan()) return;
  spline1.Start();

  SplineHandle spline2 = PlanSpline(
      [this](aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder
                 *builder) { return auto_splines_.AutoNavBounce2(builder); },
      SplineDirection::kBackward);

  if (!spline1.WaitForSplineDistanceRemaining(0.02)) return;

  if (!spline2.WaitForPlan()) return;
  spline2.Start();

  SplineHandle spline3 = PlanSpline(
      [this](aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder
                 *builder) { return auto_splines_.AutoNavBounce3(builder); },
      SplineDirection::kForward);

  if (!spline2.WaitForSplineDistanceRemaining(0.02)) return;

  if (!spline3.WaitForPlan()) return;
  spline3.Start();

  SplineHandle spline4 = PlanSpline(
      [this](aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder
                 *builder) { return auto_splines_.AutoNavBounce4(builder); },
      SplineDirection::kBackward);

  if (!spline3.WaitForSplineDistanceRemaining(0.02)) return;

  if (!spline4.WaitForPlan()) return;
  spline4.Start();

  if (!spline4.WaitForSplineDistanceRemaining(0.02)) return;
}

void AutonomousActor::AutoNavBarrel() {
  SplineHandle spline1 = PlanSpline(
      [this](aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder
                 *builder) {
        flatbuffers::Offset<frc971::MultiSpline> target_spline;
        target_spline = auto_splines_.AutoNavBarrel(builder);
        const frc971::MultiSpline *const spline =
            flatbuffers::GetTemporaryPointer(*builder->fbb(), target_spline);
        SendStartingPosition(CHECK_NOTNULL(spline));
        return target_spline;
      },
      SplineDirection::kForward);

  if (!spline1.WaitForPlan()) return;
  spline1.Start();

  if (!spline1.WaitForSplineDistanceRemaining(0.02)) return;
}

void AutonomousActor::AutoNavSlalom() {
  SplineHandle spline1 = PlanSpline(
      [this](aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder
                 *builder) {
        flatbuffers::Offset<frc971::MultiSpline> target_spline;
        target_spline = auto_splines_.AutoNavSlalom(builder);
        const frc971::MultiSpline *const spline =
            flatbuffers::GetTemporaryPointer(*builder->fbb(), target_spline);
        SendStartingPosition(CHECK_NOTNULL(spline));
        return target_spline;
      },
      SplineDirection::kForward);

  if (!spline1.WaitForPlan()) return;
  spline1.Start();

  if (!spline1.WaitForSplineDistanceRemaining(0.02)) return;
}

void AutonomousActor::SplineAuto() {
  SplineHandle spline1 = PlanSpline(
      [this](aos::Sender<frc971::control_loops::drivetrain::Goal>::Builder
                 *builder) {
        flatbuffers::Offset<frc971::MultiSpline> target_spline;
        target_spline = auto_splines_.TestSpline(builder);
        const frc971::MultiSpline *const spline =
            flatbuffers::GetTemporaryPointer(*builder->fbb(), target_spline);
        SendStartingPosition(CHECK_NOTNULL(spline));
        return target_spline;
      },
      SplineDirection::kForward);

  if (!spline1.WaitForPlan()) return;
  spline1.Start();

  if (!spline1.WaitForSplineDistanceRemaining(0.02)) return;
}

void AutonomousActor::SendStartingPosition(
    const frc971::MultiSpline *const spline) {
  float x = spline->spline_x()->Get(0);
  float y = spline->spline_y()->Get(0);

  Eigen::Matrix<double, 2, 6> control_points;
  for (size_t ii = 0; ii < 6; ++ii) {
    control_points(0, ii) = spline->spline_x()->Get(ii);
    control_points(1, ii) = spline->spline_y()->Get(ii);
  }

  frc971::control_loops::drivetrain::Spline spline_object(control_points);

  SendStartingPosition(x, y, spline_object.Theta(0));
}

ProfileParametersT MakeProfileParametersT(const float max_velocity,
                                          const float max_acceleration) {
  ProfileParametersT params;
  params.max_velocity = max_velocity;
  params.max_acceleration = max_acceleration;
  return params;
}

bool AutonomousActor::DriveFwd() {
  SendStartingPosition(0, 0, 0);
  const ProfileParametersT kDrive = MakeProfileParametersT(0.3f, 1.0f);
  const ProfileParametersT kTurn = MakeProfileParametersT(5.0f, 15.0f);
  StartDrive(1.0, 0.0, kDrive, kTurn);
  return WaitForDriveDone();
}

void AutonomousActor::SendSuperstructureGoal() {

  auto builder = superstructure_goal_sender_.MakeBuilder();

  flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
      intake_offset;

  {
    StaticZeroingSingleDOFProfiledSubsystemGoal::Builder intake_builder =
        builder.MakeBuilder<StaticZeroingSingleDOFProfiledSubsystemGoal>();

    frc971::ProfileParameters::Builder profile_params_builder =
        builder.MakeBuilder<frc971::ProfileParameters>();
    profile_params_builder.add_max_velocity(10.0);
    profile_params_builder.add_max_acceleration(30.0);
    flatbuffers::Offset<frc971::ProfileParameters> profile_params_offset =
        profile_params_builder.Finish();
    intake_builder.add_unsafe_goal(intake_goal_);
    intake_builder.add_profile_params(profile_params_offset);
    intake_offset = intake_builder.Finish();
  }

  superstructure::Goal::Builder superstructure_builder =
        builder.MakeBuilder<superstructure::Goal>();

  superstructure_builder.add_intake(intake_offset);
  superstructure_builder.add_roller_voltage(roller_voltage_);
  superstructure_builder.add_roller_speed_compensation(kRollerSpeedCompensation);

  if (!builder.Send(superstructure_builder.Finish())) {
    AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
  }

}
}  // namespace actors
}  // namespace y2020
