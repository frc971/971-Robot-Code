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
DEFINE_bool(target_aligned, true,
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
      superstructure_status_fetcher_(
          event_loop->MakeFetcher<y2020::control_loops::superstructure::Status>(
              "/superstructure")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      robot_state_fetcher_(event_loop->MakeFetcher<aos::RobotState>("/aos")),
      auto_splines_() {
  set_max_drivetrain_voltage(12.0);
  replan_timer_ = event_loop->AddTimer([this]() { Replan(); });
  event_loop->OnRun([this, event_loop]() {
    replan_timer_->Setup(event_loop->monotonic_now());
    button_poll_->Setup(event_loop->monotonic_now(), chrono::milliseconds(50));
  });

  button_poll_ = event_loop->AddTimer([this]() {
    const aos::monotonic_clock::time_point now =
        this->event_loop()->context().monotonic_event_time;
    if (robot_state_fetcher_.Fetch()) {
      if (robot_state_fetcher_->user_button()) {
        user_indicated_safe_to_reset_ = true;
        MaybeSendStartingPosition();
      }
    }
    if (joystick_state_fetcher_.Fetch()) {
      if (joystick_state_fetcher_->has_alliance() &&
          (joystick_state_fetcher_->alliance() != alliance_)) {
        alliance_ = joystick_state_fetcher_->alliance();
        is_planned_ = false;
        // Only kick the planning out by 2 seconds. If we end up enabled in that
        // second, then we will kick it out further based on the code below.
        replan_timer_->Setup(now + std::chrono::seconds(2));
      }
      if (joystick_state_fetcher_->enabled()) {
        if (!is_planned_) {
          // Only replan once we've been disabled for 5 seconds.
          replan_timer_->Setup(now + std::chrono::seconds(5));
        }
      }
    }
  });
}

void AutonomousActor::MaybeSendStartingPosition() {
  if (is_planned_ && user_indicated_safe_to_reset_ &&
      !sent_starting_position_) {
    CHECK(starting_position_);
    SendStartingPosition(starting_position_.value());
  }
}

void AutonomousActor::Replan() {
  LOG(INFO) << "Alliance " << static_cast<int>(alliance_);
  if (alliance_ == aos::Alliance::kInvalid) {
    return;
  }
  sent_starting_position_ = false;
  if (FLAGS_spline_auto) {
    test_spline_ = PlanSpline(std::bind(&AutonomousSplines::TestSpline,
                                        &auto_splines_, std::placeholders::_1),
                              SplineDirection::kForward);
    starting_position_ = test_spline_->starting_position();
  } else if (FLAGS_target_aligned) {
    target_aligned_splines_ = {
        PlanSpline(std::bind(&AutonomousSplines::TargetAligned1, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kForward),
        PlanSpline(std::bind(&AutonomousSplines::TargetAligned2, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kBackward),
        PlanSpline(std::bind(&AutonomousSplines::TargetAligned3, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kForward)};
    starting_position_ = target_aligned_splines_.value()[0].starting_position();
    CHECK(starting_position_);
  } else if (FLAGS_target_offset) {
    target_offset_splines_ = {
        PlanSpline(std::bind(&AutonomousSplines::TargetOffset1, &auto_splines_,
                             std::placeholders::_1),
                   SplineDirection::kForward),
        PlanSpline(std::bind(&AutonomousSplines::TargetOffset2, &auto_splines_,
                             std::placeholders::_1),
                   SplineDirection::kBackward)};
    starting_position_ = target_offset_splines_.value()[0].starting_position();
  } else {
    starting_position_ = Eigen::Vector3d::Zero();
  }

  is_planned_ = true;

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
    CHECK(starting_position_);
    SendStartingPosition(starting_position_.value());
  }
  // Clear this so that we don't accidentally resend things as soon as we replan
  // later.
  user_indicated_safe_to_reset_ = false;
  is_planned_ = false;
  starting_position_.reset();

  AOS_LOG(INFO, "Params are %d\n", params->mode());
  if (alliance_ == aos::Alliance::kInvalid) {
    AOS_LOG(INFO, "Aborting autonomous due to invalid alliance selection.");
    return false;
  }
  if (FLAGS_spline_auto) {
    SplineAuto();
  } else if (FLAGS_target_aligned) {
    TargetAligned();
  } else if (FLAGS_target_offset) {
    TargetOffset();
  } else if (FLAGS_just_shoot) {
    JustShoot();
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
  LOG(INFO) << "User button pressed, x: " << start(0) << " y: " << start(1)
            << " theta: " << start(2);
  if (!builder.Send(localizer_control_builder.Finish())) {
    AOS_LOG(ERROR, "Failed to reset localizer.\n");
  }
}

void AutonomousActor::TargetAligned() {
  aos::monotonic_clock::time_point start_time = aos::monotonic_clock::now();
  CHECK(target_aligned_splines_);
  auto &splines = *target_aligned_splines_;

  // Spin up.
  set_shooting(true);
  set_preloading(true);
  set_shooter_tracking(true);
  SendSuperstructureGoal();
  if (!WaitForBallsShot(3)) return;
  LOG(INFO) << "Shot balls";
  set_shooter_tracking(false);

  // Drive and intake 3 balls in front of the trench run
  set_shooting(false);
  ExtendIntake();
  SendSuperstructureGoal();

  if (!splines[0].WaitForPlan()) return;
  splines[0].Start();

  if (!splines[0].WaitForSplineDistanceRemaining(0.02)) return;

  std::this_thread::sleep_for(chrono::milliseconds(200));
  RetractIntake();

  // Drive back to shooting position
  if (!splines[1].WaitForPlan()) return;
  splines[1].Start();

  if (!splines[1].WaitForSplineDistanceRemaining(2.0)) return;
  // Reverse the rollers for a moment to try to unjam any jammed balls.  Since
  // we are moving here, this is free to try.
  set_roller_voltage(-12.0);
  std::this_thread::sleep_for(chrono::milliseconds(300));
  set_roller_voltage(0.0);

  // Once we come to a stop, give the robot a moment to settle down.  This makes
  // the shot more accurate.
  if (!splines[1].WaitForSplineDistanceRemaining(0.02)) return;
  set_shooter_tracking(true);
  std::this_thread::sleep_for(chrono::milliseconds(1500));
  set_shooting(true);
  const int balls = Balls();

  SendSuperstructureGoal();

  SendSuperstructureGoal();

  if (!WaitUntilAbsoluteBallsShot(3 + balls)) return;

  set_shooting(false);
  set_roller_voltage(0.0);
  set_shooter_tracking(false);
  set_preloading(false);
  SendSuperstructureGoal();

  // Drive close to the rendezvous point in the center of the field so that the
  // driver can intake balls there right after auto ends.
  if (!splines[2].WaitForPlan()) return;
  splines[2].Start();

  if (!splines[2].WaitForSplineDistanceRemaining(0.02)) return;

  LOG(INFO) << "Took "
            << chrono::duration<double>(aos::monotonic_clock::now() -
                                        start_time)
                   .count();
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
  superstructure_builder.add_intake_preloading(preloading_);
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
  set_intake_goal(1.30);
  set_roller_voltage(6.0);
  SendSuperstructureGoal();
}

void AutonomousActor::RetractIntake() {
  set_intake_goal(-0.89);
  set_roller_voltage(6.0);
  SendSuperstructureGoal();
}

int AutonomousActor::Balls() {
  superstructure_status_fetcher_.Fetch();
  CHECK(superstructure_status_fetcher_.get() != nullptr);
  return superstructure_status_fetcher_->shooter()->balls_shot();
}

bool AutonomousActor::WaitUntilAbsoluteBallsShot(int absolute_balls) {
  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      frc971::controls::kLoopFrequency / 2);
  superstructure_status_fetcher_.Fetch();
  CHECK(superstructure_status_fetcher_.get() != nullptr);
  int last_balls = superstructure_status_fetcher_->shooter()->balls_shot();
  LOG(INFO) << "Waiting for balls, started with " << absolute_balls;
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    superstructure_status_fetcher_.Fetch();
    CHECK(superstructure_status_fetcher_.get() != nullptr);
    if (superstructure_status_fetcher_->shooter()->balls_shot() != last_balls) {
      LOG(INFO) << "Shot "
                << superstructure_status_fetcher_->shooter()->balls_shot() -
                       last_balls
                << " balls, now at "
                << superstructure_status_fetcher_->shooter()->balls_shot();
    }
    if (superstructure_status_fetcher_->shooter()->balls_shot() >=
        absolute_balls) {
      return true;
    }

    last_balls = superstructure_status_fetcher_->shooter()->balls_shot();
  }
}

bool AutonomousActor::WaitForBallsShot(int num_wanted) {
  return WaitUntilAbsoluteBallsShot(Balls() + num_wanted);
}

}  // namespace actors
}  // namespace y2020
