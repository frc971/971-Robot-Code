#include "y2020/actors/autonomous_actor.h"

#include <chrono>
#include <cinttypes>
#include <cmath>

#include "aos/logging/logging.h"
#include "aos/util/math.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2020/actors/auto_splines.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"

DEFINE_bool(spline_auto, false, "If true, define a spline autonomous mode");
DEFINE_bool(ignore_vision, false, "If true, ignore vision");
DEFINE_bool(galactic_search, false,
            "If true, do the galactic search autonomous");
DEFINE_bool(bounce, false, "If true, run the AutoNav Bounce autonomous");
DEFINE_bool(barrel, false, "If true, run the AutoNav Barrel autonomous");
DEFINE_bool(slalom, true, "If true, run the AutoNav Slalom autonomous");

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
      superstructure_goal_sender_(
          event_loop->MakeSender<control_loops::superstructure::Goal>(
              "/superstructure")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      path_fetcher_(event_loop->MakeFetcher<y2020::vision::GalacticSearchPath>(
          "/pi2/camera")),
      superstructure_status_fetcher_(
          event_loop->MakeFetcher<y2020::control_loops::superstructure::Status>(
              "/superstructure")),
      auto_splines_() {
  set_max_drivetrain_voltage(2.0);
  replan_timer_ = event_loop->AddTimer([this]() { Replan(); });
  event_loop->OnRun([this, event_loop]() {
    replan_timer_->Setup(event_loop->monotonic_now());
  });
}

void AutonomousActor::Replan() {
  if (FLAGS_galactic_search) {
    galactic_search_splines_ = {
        .red_a = PlanSpline(std::bind(&AutonomousSplines::SplineRedA,
                                      &auto_splines_, std::placeholders::_1),
                            SplineDirection::kForward),
        .red_b = PlanSpline(std::bind(&AutonomousSplines::SplineRedB,
                                      &auto_splines_, std::placeholders::_1),
                            SplineDirection::kForward),
        .blue_a = PlanSpline(std::bind(&AutonomousSplines::SplineBlueA,
                                       &auto_splines_, std::placeholders::_1),
                             SplineDirection::kForward),
        .blue_b = PlanSpline(std::bind(&AutonomousSplines::SplineBlueB,
                                       &auto_splines_, std::placeholders::_1),
                             SplineDirection::kForward)};
  } else if (FLAGS_bounce) {
    bounce_splines_ = {
        PlanSpline(std::bind(&AutonomousSplines::AutoNavBounce1, &auto_splines_,
                             std::placeholders::_1),
                   SplineDirection::kForward),
        PlanSpline(std::bind(&AutonomousSplines::AutoNavBounce2, &auto_splines_,
                             std::placeholders::_1),
                   SplineDirection::kBackward),
        PlanSpline(std::bind(&AutonomousSplines::AutoNavBounce3, &auto_splines_,
                             std::placeholders::_1),
                   SplineDirection::kForward),
        PlanSpline(std::bind(&AutonomousSplines::AutoNavBounce4, &auto_splines_,
                             std::placeholders::_1),
                   SplineDirection::kBackward)};
  } else if (FLAGS_barrel) {
    barrel_spline_ =
        PlanSpline(std::bind(&AutonomousSplines::AutoNavBarrel, &auto_splines_,
                             std::placeholders::_1),
                   SplineDirection::kForward);
  } else if (FLAGS_slalom) {
    slalom_spline_ =
        PlanSpline(std::bind(&AutonomousSplines::AutoNavSlalom, &auto_splines_,
                             std::placeholders::_1),
                   SplineDirection::kForward);
  } else if (FLAGS_spline_auto) {
    test_spline_ = PlanSpline(std::bind(&AutonomousSplines::TestSpline,
                                        &auto_splines_, std::placeholders::_1),
                              SplineDirection::kForward);
  }
}

void AutonomousActor::Reset() {
  InitializeEncoders();
  ResetDrivetrain();
  RetractIntake();

  joystick_state_fetcher_.Fetch();
  CHECK(joystick_state_fetcher_.get() != nullptr)
      << "Expect at least one JoystickState message before running auto...";
  alliance_ = joystick_state_fetcher_->alliance();
}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  Reset();

  // Queue up a replan to occur as soon as this action completes.
  // TODO(james): Modify this so we don't replan during teleop.
  replan_timer_->Setup(monotonic_now());

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

void AutonomousActor::SendStartingPosition(const Eigen::Vector3d &start) {
  // Set up the starting position for the blue alliance.

  // TODO(james): Resetting the localizer breaks the left/right statespace
  // controller.  That is a bug, but we can fix that later by not resetting.
  auto builder = localizer_control_sender_.MakeBuilder();

  LocalizerControl::Builder localizer_control_builder =
      builder.MakeBuilder<LocalizerControl>();
  localizer_control_builder.add_x(start(0));
  localizer_control_builder.add_y(start(1));
  localizer_control_builder.add_theta(start(2));
  localizer_control_builder.add_theta_uncertainty(0.00001);
  if (!builder.Send(localizer_control_builder.Finish())) {
    AOS_LOG(ERROR, "Failed to reset localizer.\n");
  }
}

void AutonomousActor::GalacticSearch() {
  CHECK(galactic_search_splines_);

  path_fetcher_.Fetch();
  SplineHandle *spline = nullptr;
  if (path_fetcher_.get()) {
    if (path_fetcher_->alliance() == y2020::vision::Alliance::kUnknown) {
      AOS_LOG(ERROR, "The galactic search path is unknown, doing nothing.");
      return;
    }
    if (path_fetcher_->alliance() == y2020::vision::Alliance::kRed) {
      if (path_fetcher_->letter() == y2020::vision::Letter::kA) {
        LOG(INFO) << "Red A";
        spline = &galactic_search_splines_->red_a;
      } else {
        LOG(INFO) << "Red B";
        CHECK(path_fetcher_->letter() == y2020::vision::Letter::kB);
        spline = &galactic_search_splines_->red_b;
      }
    } else {
      if (path_fetcher_->letter() == y2020::vision::Letter::kA) {
        LOG(INFO) << "Blue A";
        spline = &galactic_search_splines_->blue_a;
      } else {
        LOG(INFO) << "Blue B";
        CHECK(path_fetcher_->letter() == y2020::vision::Letter::kB);
        spline = &galactic_search_splines_->blue_b;
      }
    }
  }
  if (FLAGS_ignore_vision) {
    LOG(INFO) << "Forcing Red B";
    spline = &galactic_search_splines_->red_b;
  }

  CHECK(spline != nullptr)
      << "Expect at least one GalacticSearchPath message before running "
         "auto...";

  SendStartingPosition(spline->starting_position());

  set_intake_goal(1.25);
  set_roller_voltage(12.0);
  SendSuperstructureGoal();

  if (!spline->WaitForPlan()) return;
  spline->Start();

  if (!spline->WaitForSplineDistanceRemaining(0.02)) return;
  RetractIntake();
}

void AutonomousActor::AutoNavBounce() {
  CHECK(bounce_splines_);

  auto &splines = *bounce_splines_;

  SendStartingPosition(splines[0].starting_position());

  if (!splines[0].WaitForPlan()) return;
  splines[0].Start();

  if (!splines[0].WaitForSplineDistanceRemaining(0.02)) return;

  if (!splines[1].WaitForPlan()) return;
  splines[1].Start();

  if (!splines[1].WaitForSplineDistanceRemaining(0.02)) return;

  if (!splines[2].WaitForPlan()) return;
  splines[2].Start();

  if (!splines[2].WaitForSplineDistanceRemaining(0.02)) return;

  if (!splines[3].WaitForPlan()) return;
  splines[3].Start();

  if (!splines[3].WaitForSplineDistanceRemaining(0.02)) return;
}

void AutonomousActor::AutoNavBarrel() {
  CHECK(barrel_spline_);

  SendStartingPosition(barrel_spline_->starting_position());

  if (!barrel_spline_->WaitForPlan()) return;
  barrel_spline_->Start();

  if (!barrel_spline_->WaitForSplineDistanceRemaining(0.02)) return;
}

void AutonomousActor::AutoNavSlalom() {
  CHECK(slalom_spline_);

  SendStartingPosition(slalom_spline_->starting_position());

  if (!slalom_spline_->WaitForPlan()) return;
  slalom_spline_->Start();

  if (!slalom_spline_->WaitForSplineDistanceRemaining(0.02)) return;
}

void AutonomousActor::SplineAuto() {
  CHECK(test_spline_);

  SendStartingPosition(test_spline_->starting_position());

  if (!test_spline_->WaitForPlan()) return;
  test_spline_->Start();

  if (!test_spline_->WaitForSplineDistanceRemaining(0.02)) return;
}

ProfileParametersT MakeProfileParametersT(const float max_velocity,
                                          const float max_acceleration) {
  ProfileParametersT params;
  params.max_velocity = max_velocity;
  params.max_acceleration = max_acceleration;
  return params;
}

bool AutonomousActor::DriveFwd() {
  SendStartingPosition({0, 0, 0});
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
    profile_params_builder.add_max_velocity(20.0);
    profile_params_builder.add_max_acceleration(60.0);
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
  superstructure_builder.add_roller_speed_compensation(
      kRollerSpeedCompensation);

  if (!builder.Send(superstructure_builder.Finish())) {
    AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
  }
}

void AutonomousActor::RetractIntake() {
  set_intake_goal(-0.89);
  set_roller_voltage(0.0);
  SendSuperstructureGoal();
}

bool AutonomousActor::WaitForBallsShot(int num_wanted) {
  superstructure_status_fetcher_.Fetch();
  CHECK(superstructure_status_fetcher_.get() != nullptr);
  const int initial_balls_shot =
      superstructure_status_fetcher_->shooter()->balls_shot();
  int balls_shot = initial_balls_shot;

  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      frc971::controls::kLoopFrequency / 2);
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    superstructure_status_fetcher_.Fetch();
    balls_shot = superstructure_status_fetcher_->shooter()->balls_shot();
    if ((balls_shot - initial_balls_shot) >= num_wanted) {
      return true;
    }
  }
}

}  // namespace actors
}  // namespace y2020
