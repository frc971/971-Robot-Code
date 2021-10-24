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
DEFINE_bool(target_aligned, false,
            "If true, run the Infinite Recharge autonomous that starts aligned "
            "with the target");
DEFINE_bool(target_offset, false,
            "If true, run the Infinite Recharge autonomous that starts offset "
            "from the target");
DEFINE_bool(just_shoot, false,
            "If true, run the autonomous that just shoots balls.");

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
      superstructure_status_fetcher_(
          event_loop->MakeFetcher<y2020::control_loops::superstructure::Status>(
              "/superstructure")),
      auto_splines_() {
  set_max_drivetrain_voltage(2.0);
  replan_timer_ = event_loop->AddTimer([this]() { Replan(); });
  event_loop->OnRun([this, event_loop]() {
    replan_timer_->Setup(event_loop->monotonic_now());
  });
  event_loop->MakeWatcher("/aos", [this](const aos::RobotState &msg) {
    if (msg.user_button()) {
      user_indicated_safe_to_reset_ = true;
      MaybeSendStartingPosition();
    }
  });
  event_loop->MakeWatcher("/aos", [this](const aos::JoystickState &msg) {
    if (msg.has_alliance() && (msg.alliance() != alliance_)) {
      alliance_ = msg.alliance();
      Replan();
    }
  });
}

void AutonomousActor::MaybeSendStartingPosition() {
  if (user_indicated_safe_to_reset_ && !sent_starting_position_) {
    CHECK(starting_position_);
    SendStartingPosition(starting_position_.value());
  }
}

void AutonomousActor::Replan() {
  sent_starting_position_ = false;
  if (FLAGS_spline_auto) {
    test_spline_ = PlanSpline(std::bind(&AutonomousSplines::TestSpline,
                                        &auto_splines_, std::placeholders::_1),
                              SplineDirection::kForward);
    starting_position_ = test_spline_->starting_position();
  } else if (FLAGS_target_offset) {
    target_offset_splines_ = {
        PlanSpline(std::bind(&AutonomousSplines::TargetOffset1, &auto_splines_,
                             std::placeholders::_1),
                   SplineDirection::kForward),
        PlanSpline(std::bind(&AutonomousSplines::TargetOffset2, &auto_splines_,
                             std::placeholders::_1),
                   SplineDirection::kBackward)};
    starting_position_ = target_offset_splines_.value()[0].starting_position();
  } else if (FLAGS_target_aligned) {
    target_aligned_splines_ = {
        PlanSpline(std::bind(&AutonomousSplines::TargetAligned1, &auto_splines_,
                             std::placeholders::_1),
                   SplineDirection::kForward),
        PlanSpline(std::bind(&AutonomousSplines::TargetAligned2, &auto_splines_,
                             std::placeholders::_1),
                   SplineDirection::kBackward)};
    starting_position_ = target_aligned_splines_.value()[0].starting_position();
  } else {
    starting_position_ = Eigen::Vector3d::Zero();
  }
  MaybeSendStartingPosition();
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
  if (!user_indicated_safe_to_reset_) {
    AOS_LOG(WARNING, "Didn't send starting position prior to starting auto.");
    SendStartingPosition(starting_position_.value());
  }

  // Queue up a replan to occur as soon as this action completes.
  // TODO(james): Modify this so we don't replan during teleop.
  replan_timer_->Setup(monotonic_now());

  AOS_LOG(INFO, "Params are %d\n", params->mode());
  if (alliance_ == aos::Alliance::kInvalid) {
    AOS_LOG(INFO, "Aborting autonomous due to invalid alliance selection.");
    return false;
  }
  if (FLAGS_target_aligned) {
    TargetAligned();
  } else if (FLAGS_target_offset) {
    TargetOffset();
  } else if (FLAGS_just_shoot) {
    JustShoot();
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

void AutonomousActor::TargetAligned() {
  CHECK(target_aligned_splines_);
  auto &splines = *target_aligned_splines_;

  // shoot pre-loaded balls
  set_shooter_tracking(true);
  set_shooting(true);
  SendSuperstructureGoal();

  if (!WaitForBallsShot(3)) return;

  set_shooting(false);
  SendSuperstructureGoal();

  ExtendIntake();

  // pickup 3 more balls
  if (!splines[0].WaitForPlan()) return;
  splines[0].Start();

  if (!splines[0].WaitForSplineDistanceRemaining(0.02)) return;
  RetractIntake();

  if (!splines[1].WaitForPlan()) return;
  splines[1].Start();

  if (!splines[1].WaitForSplineDistanceRemaining(0.02)) return;

  // shoot the new balls in front of the goal.
  set_shooting(true);
  SendSuperstructureGoal();

  if (!WaitForBallsShot(3)) return;

  set_shooting(false);
  set_shooter_tracking(false);
  SendSuperstructureGoal();
}

void AutonomousActor::JustShoot() {
  // shoot pre-loaded balls
  set_shooter_tracking(true);
  set_shooting(true);
  SendSuperstructureGoal();

  if (!WaitForBallsShot(3)) return;

  set_shooting(false);
  set_shooter_tracking(true);
  SendSuperstructureGoal();
}

void AutonomousActor::TargetOffset() {
  CHECK(target_offset_splines_);
  auto &splines = *target_offset_splines_;

  // spin up shooter
  set_shooter_tracking(true);
  SendSuperstructureGoal();
  ExtendIntake();

  // pickup 2 more balls in front of the trench run
  if (!splines[0].WaitForPlan()) return;
  splines[0].Start();

  if (!splines[0].WaitForSplineDistanceRemaining(0.02)) return;
  RetractIntake();

  if (!splines[1].WaitForPlan()) return;
  splines[1].Start();

  if (!splines[1].WaitForSplineDistanceRemaining(0.02)) return;

  // shoot the balls from in front of the goal.
  set_shooting(true);
  SendSuperstructureGoal();

  if (!WaitForBallsShot(5)) return;

  set_shooting(false);
  set_shooter_tracking(false);
  SendSuperstructureGoal();
}

void AutonomousActor::SplineAuto() {
  CHECK(test_spline_);

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
  superstructure_builder.add_intake_preloading(intake_preloading_);
  superstructure_builder.add_roller_voltage(roller_voltage_);
  superstructure_builder.add_roller_speed_compensation(
      kRollerSpeedCompensation);
  superstructure_builder.add_hood_tracking(shooter_tracking_);
  superstructure_builder.add_turret_tracking(shooter_tracking_);
  superstructure_builder.add_shooter_tracking(shooter_tracking_);
  superstructure_builder.add_shooting(shooting_);

  if (!builder.Send(superstructure_builder.Finish())) {
    AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
  }
}

void AutonomousActor::ExtendIntake() {
  set_intake_goal(1.25);
  set_roller_voltage(12.0);
  set_intake_preloading(true);
  SendSuperstructureGoal();
}

void AutonomousActor::RetractIntake() {
  set_intake_goal(-0.89);
  set_roller_voltage(0.0);
  set_intake_preloading(false);
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
