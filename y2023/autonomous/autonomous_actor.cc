#include "y2023/autonomous/autonomous_actor.h"

#include <chrono>
#include <cinttypes>
#include <cmath>

#include "aos/logging/logging.h"
#include "aos/util/math.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2023/autonomous/auto_splines.h"
#include "y2023/constants.h"
#include "y2023/control_loops/drivetrain/drivetrain_base.h"
#include "y2023/control_loops/superstructure/arm/generated_graph.h"

DEFINE_bool(spline_auto, false, "Run simple test S-spline auto mode.");
DEFINE_bool(charged_up, true, "If true run charged up autonomous mode");
DEFINE_bool(charged_up_cable, false, "If true run cable side autonomous mode");

namespace y2023 {
namespace autonomous {

using ::frc971::ProfileParametersT;

ProfileParametersT MakeProfileParameters(float max_velocity,
                                         float max_acceleration) {
  ProfileParametersT result;
  result.max_velocity = max_velocity;
  result.max_acceleration = max_acceleration;
  return result;
}

using ::aos::monotonic_clock;
using frc971::CreateProfileParameters;
using ::frc971::ProfileParametersT;
using frc971::control_loops::CreateStaticZeroingSingleDOFProfiledSubsystemGoal;
using frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal;
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
      robot_state_fetcher_(event_loop->MakeFetcher<aos::RobotState>("/aos")),
      auto_splines_(),
      arm_goal_position_(control_loops::superstructure::arm::StartingIndex()),
      superstructure_goal_sender_(
          event_loop->MakeSender<::y2023::control_loops::superstructure::Goal>(
              "/superstructure")),
      superstructure_status_fetcher_(
          event_loop
              ->MakeFetcher<::y2023::control_loops::superstructure::Status>(
                  "/superstructure")),
      points_(control_loops::superstructure::arm::PointList()) {
  drivetrain_status_fetcher_.Fetch();
  replan_timer_ = event_loop->AddTimer([this]() { Replan(); });

  event_loop->OnRun([this, event_loop]() {
    replan_timer_->Setup(event_loop->monotonic_now());
    button_poll_->Setup(event_loop->monotonic_now(), chrono::milliseconds(50));
  });

  // TODO(james): Really need to refactor this code since we keep using it.
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
        // Only kick the planning out by 2 seconds. If we end up enabled in
        // that second, then we will kick it out further based on the code
        // below.
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

void AutonomousActor::Replan() {
  if (!drivetrain_status_fetcher_.Fetch()) {
    replan_timer_->Setup(event_loop()->monotonic_now() + chrono::seconds(1));
    AOS_LOG(INFO, "Drivetrain not up, replanning in 1 second");
    return;
  }

  if (alliance_ == aos::Alliance::kInvalid) {
    return;
  }
  sent_starting_position_ = false;
  if (FLAGS_spline_auto) {
    test_spline_ =
        PlanSpline(std::bind(&AutonomousSplines::TestSpline, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kForward);

    starting_position_ = test_spline_->starting_position();
  } else if (FLAGS_charged_up) {
    AOS_LOG(INFO, "Charged up replanning!");
    charged_up_splines_ = {
        PlanSpline(std::bind(&AutonomousSplines::Spline1, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kBackward),
        PlanSpline(std::bind(&AutonomousSplines::Spline2, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kForward),
        PlanSpline(std::bind(&AutonomousSplines::Spline3, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kBackward),
        PlanSpline(std::bind(&AutonomousSplines::Spline4, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kForward)};

    starting_position_ = charged_up_splines_.value()[0].starting_position();
    CHECK(starting_position_);
  } else if (FLAGS_charged_up_cable) {
    charged_up_cable_splines_ = {
        PlanSpline(std::bind(&AutonomousSplines::SplineCable1, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kBackward),
        PlanSpline(std::bind(&AutonomousSplines::SplineCable2, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kForward),
        PlanSpline(std::bind(&AutonomousSplines::SplineCable3, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kBackward),
        PlanSpline(std::bind(&AutonomousSplines::SplineCable4, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kForward)};

    starting_position_ =
        charged_up_cable_splines_.value()[0].starting_position();
    CHECK(starting_position_);
  }

  is_planned_ = true;

  MaybeSendStartingPosition();
}

void AutonomousActor::MaybeSendStartingPosition() {
  if (is_planned_ && user_indicated_safe_to_reset_ &&
      !sent_starting_position_) {
    CHECK(starting_position_);
    SendStartingPosition(starting_position_.value());
  }
}

void AutonomousActor::Reset() {
  InitializeEncoders();
  ResetDrivetrain();

  joystick_state_fetcher_.Fetch();
  CHECK(joystick_state_fetcher_.get() != nullptr)
      << "Expect at least one JoystickState message before running auto...";
  alliance_ = joystick_state_fetcher_->alliance();

  wrist_goal_ = 0.0;
  roller_goal_ = control_loops::superstructure::RollerGoal::IDLE;
  arm_goal_position_ = control_loops::superstructure::arm::StartingIndex();
  preloaded_ = false;
  SendSuperstructureGoal();
}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  Reset();

  AOS_LOG(INFO, "Params are %d\n", params->mode());

  if (!user_indicated_safe_to_reset_) {
    AOS_LOG(WARNING, "Didn't send starting position prior to starting auto.");
    CHECK(starting_position_);
    SendStartingPosition(starting_position_.value());
  }
  // Clear this so that we don't accidentally resend things as soon as we
  // replan later.
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
  } else if (FLAGS_charged_up) {
    ChargedUp();
  } else if (FLAGS_charged_up_cable) {
    ChargedUpCableSide();
  } else {
    AOS_LOG(WARNING, "No auto mode selected.");
  }
  return true;
}

void AutonomousActor::SplineAuto() {
  CHECK(test_spline_);

  if (!test_spline_->WaitForPlan()) return;
  test_spline_->Start();

  if (!test_spline_->WaitForSplineDistanceRemaining(0.02)) return;
}

void AutonomousActor::SendStartingPosition(const Eigen::Vector3d &start) {
  // Set up the starting position for the blue alliance.

  auto builder = localizer_control_sender_.MakeBuilder();

  LocalizerControl::Builder localizer_control_builder =
      builder.MakeBuilder<LocalizerControl>();
  localizer_control_builder.add_x(start(0));
  localizer_control_builder.add_y(start(1));
  localizer_control_builder.add_theta(start(2));
  localizer_control_builder.add_theta_uncertainty(0.00001);
  AOS_LOG(INFO, "User button pressed, x: %f y: %f theta: %f", start(0),
          start(1), start(2));
  if (builder.Send(localizer_control_builder.Finish()) !=
      aos::RawSender::Error::kOk) {
    AOS_LOG(ERROR, "Failed to reset localizer.\n");
  }
}

// Charged Up 3 Game Object Autonomous (non-cable side)
void AutonomousActor::ChargedUp() {
  aos::monotonic_clock::time_point start_time = aos::monotonic_clock::now();

  CHECK(charged_up_splines_);

  auto &splines = *charged_up_splines_;

  AOS_LOG(INFO, "Going to preload");

  // Tell the superstructure a cone was preloaded
  if (!WaitForPreloaded()) return;
  AOS_LOG(INFO, "Moving arm");

  // Place first cone on mid level
  MidConeScore();

  // Wait until the arm is at the goal to spit
  if (!WaitForArmGoal(0.10)) return;
  Spit();
  if (!WaitForArmGoal(0.01)) return;

  std::this_thread::sleep_for(chrono::milliseconds(100));

  AOS_LOG(
      INFO, "Placed first cone %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  // Drive and intake the cube nearest to the starting zone
  if (!splines[0].WaitForPlan()) return;
  splines[0].Start();

  // Move arm into position to pickup a cube and start cube intake
  PickupCube();

  std::this_thread::sleep_for(chrono::milliseconds(500));

  IntakeCube();

  AOS_LOG(
      INFO, "Turning on rollers %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  if (!splines[0].WaitForSplineDistanceRemaining(0.02)) return;

  AOS_LOG(
      INFO, "Got there %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  // Drive back to grid and place cube on high level
  if (!splines[1].WaitForPlan()) return;
  splines[1].Start();

  std::this_thread::sleep_for(chrono::milliseconds(300));
  HighCubeScore();

  if (!splines[1].WaitForSplineDistanceRemaining(0.08)) return;
  AOS_LOG(
      INFO, "Back for first cube %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  if (!WaitForArmGoal(0.10)) return;

  AOS_LOG(
      INFO, "Arm in place for first cube %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  Spit();

  if (!splines[1].WaitForSplineDistanceRemaining(0.08)) return;

  AOS_LOG(
      INFO, "Finished spline back %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  if (!WaitForArmGoal(0.05)) return;

  AOS_LOG(
      INFO, "Placed first cube %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  // Drive and intake the cube second nearest to the starting zone
  if (!splines[2].WaitForPlan()) return;
  splines[2].Start();

  std::this_thread::sleep_for(chrono::milliseconds(200));
  PickupCube();

  std::this_thread::sleep_for(chrono::milliseconds(500));
  IntakeCube();

  if (!splines[2].WaitForSplineDistanceRemaining(0.05)) return;
  AOS_LOG(
      INFO, "Picked up second cube %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  // Drive back to grid and place object on mid level
  if (!splines[3].WaitForPlan()) return;
  splines[3].Start();

  AOS_LOG(
      INFO, "Driving back %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  MidCubeScore();

  if (!splines[3].WaitForSplineDistanceRemaining(0.07)) return;
  AOS_LOG(
      INFO, "Got back from second cube at %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  if (!WaitForArmGoal(0.05)) return;
  Spit();

  if (!splines[3].WaitForSplineDistanceRemaining(0.02)) return;

  AOS_LOG(
      INFO, "Placed second cube %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));
  InitializeEncoders();

  const ProfileParametersT kDrive = MakeProfileParameters(2.0, 4.0);
  const ProfileParametersT kTurn = MakeProfileParameters(3.0, 4.5);
  StartDrive(0.0, 0.0, kDrive, kTurn);

  std::this_thread::sleep_for(chrono::milliseconds(100));

  {
    double side_scalar = (alliance_ == aos::Alliance::kRed) ? 1.0 : -1.0;
    StartDrive(6.33 - std::abs(X()), 0.0, kDrive, kTurn);
    if (!WaitForDriveProfileNear(0.01)) return;

    AOS_LOG(
        INFO, "Done backing up %lf s\n",
        aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

    const ProfileParametersT kInPlaceTurn = MakeProfileParameters(2.5, 7.0);
    StartDrive(0.0, aos::math::NormalizeAngle(M_PI / 2.0 - Theta()), kDrive,
               kInPlaceTurn);

    std::this_thread::sleep_for(chrono::milliseconds(400));
    StopSpitting();

    AOS_LOG(
        INFO, "Roller off %lf s\n",
        aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

    if (!WaitForTurnProfileNear(0.6)) return;
    AOS_LOG(
        INFO, "Balance arm %lf s\n",
        aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

    Balance();
    if (!WaitForTurnProfileNear(0.001)) return;

    AOS_LOG(
        INFO, "Done turning %lf s\n",
        aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

    const ProfileParametersT kDrive = MakeProfileParameters(1.4, 3.0);
    const ProfileParametersT kFinalTurn = MakeProfileParameters(3.0, 4.5);
    const double kDriveDistance = 3.11;
    StartDrive(-kDriveDistance, 0.0, kDrive, kFinalTurn);

    const ProfileParametersT kFastTurn = MakeProfileParameters(5.0, 8.0);
    if (!WaitForDriveProfileNear(kDriveDistance - 0.4)) return;

    AOS_LOG(
        INFO, "Turning %lf s\n",
        aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));
    StartDrive(0.0, -side_scalar * M_PI / 2.0, kDrive, kFastTurn);
    if (!WaitForDriveProfileDone()) return;
    if (!WaitForTurnProfileDone()) return;
    AOS_LOG(
        INFO, "Done %lf s\n",
        aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));
  }
}

// Charged Up 3 Game Object Autonomous (cable side)
void AutonomousActor::ChargedUpCableSide() {
  aos::monotonic_clock::time_point start_time = aos::monotonic_clock::now();

  CHECK(charged_up_cable_splines_);

  auto &splines = *charged_up_cable_splines_;

  AOS_LOG(INFO, "Going to preload");

  // Tell the superstructure a cone was preloaded
  if (!WaitForPreloaded()) return;
  AOS_LOG(INFO, "Moving arm");

  // Place first cone on mid level
  MidConeScore();

  // Wait until the arm is at the goal to spit
  if (!WaitForArmGoal(0.10)) return;
  Spit();
  if (!WaitForArmGoal(0.01)) return;

  std::this_thread::sleep_for(chrono::milliseconds(100));

  AOS_LOG(
      INFO, "Placed first cone %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  // Drive and intake the cube nearest to the starting zone
  if (!splines[0].WaitForPlan()) return;
  splines[0].Start();

  // Move arm into position to pickup a cube and start cube intake
  PickupCube();

  std::this_thread::sleep_for(chrono::milliseconds(500));

  IntakeCube();

  AOS_LOG(
      INFO, "Turning on rollers %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  if (!splines[0].WaitForSplineDistanceRemaining(0.02)) return;

  AOS_LOG(
      INFO, "Got there %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  // Drive back to grid and place cube on high level
  if (!splines[1].WaitForPlan()) return;
  splines[1].Start();

  std::this_thread::sleep_for(chrono::milliseconds(300));
  Neutral();

  if (!splines[1].WaitForSplineDistanceTraveled(3.2)) return;
  HighCubeScore();
  AOS_LOG(
      INFO, "Extending arm %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  if (!splines[1].WaitForSplineDistanceRemaining(0.08)) return;
  AOS_LOG(
      INFO, "Back for first cube %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  if (!WaitForArmGoal(0.10)) return;

  AOS_LOG(
      INFO, "Arm in place for first cube %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  Spit();

  if (!splines[1].WaitForSplineDistanceRemaining(0.08)) return;

  AOS_LOG(
      INFO, "Finished spline back %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  if (!WaitForArmGoal(0.05)) return;

  AOS_LOG(
      INFO, "Placed first cube %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  // Drive and intake the cube second nearest to the starting zone
  if (!splines[2].WaitForPlan()) return;
  splines[2].Start();

  std::this_thread::sleep_for(chrono::milliseconds(200));
  PickupCube();

  std::this_thread::sleep_for(chrono::milliseconds(500));
  IntakeCube();

  if (!splines[2].WaitForSplineDistanceRemaining(0.05)) return;
  AOS_LOG(
      INFO, "Picked up second cube %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  // Drive back to grid and place object on mid level
  if (!splines[3].WaitForPlan()) return;
  splines[3].Start();

  std::this_thread::sleep_for(chrono::milliseconds(400));
  Neutral();

  AOS_LOG(
      INFO, "Driving back %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  if (!splines[3].WaitForSplineDistanceTraveled(3.5)) return;
  AOS_LOG(
      INFO, "Extending arm %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));
  MidCubeScore();

  if (!splines[3].WaitForSplineDistanceRemaining(0.07)) return;
  AOS_LOG(
      INFO, "Got back from second cube at %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  if (!WaitForArmGoal(0.05)) return;
  Spit();

  if (!splines[3].WaitForSplineDistanceRemaining(0.02)) return;

  AOS_LOG(
      INFO, "Placed second cube %lf s\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  std::this_thread::sleep_for(chrono::milliseconds(200));
  Neutral();
}

void AutonomousActor::SendSuperstructureGoal() {
  auto builder = superstructure_goal_sender_.MakeBuilder();

  flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
      wrist_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *builder.fbb(), wrist_goal_,
          CreateProfileParameters(*builder.fbb(), 12.0, 90.0));

  control_loops::superstructure::Goal::Builder superstructure_builder =
      builder.MakeBuilder<control_loops::superstructure::Goal>();

  superstructure_builder.add_arm_goal_position(arm_goal_position_);
  superstructure_builder.add_preloaded_with_cone(preloaded_);
  superstructure_builder.add_roller_goal(roller_goal_);
  superstructure_builder.add_wrist(wrist_offset);

  if (builder.Send(superstructure_builder.Finish()) !=
      aos::RawSender::Error::kOk) {
    AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
  }
}

[[nodiscard]] bool AutonomousActor::WaitForPreloaded() {
  set_preloaded(true);
  SendSuperstructureGoal();

  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      ActorBase::kLoopOffset);

  bool loaded = false;
  while (!loaded) {
    if (ShouldCancel()) {
      return false;
    }

    phased_loop.SleepUntilNext();
    superstructure_status_fetcher_.Fetch();
    CHECK(superstructure_status_fetcher_.get() != nullptr);

    loaded = (superstructure_status_fetcher_->end_effector_state() ==
              control_loops::superstructure::EndEffectorState::LOADED);
  }

  set_preloaded(false);
  SendSuperstructureGoal();

  return true;
}

void AutonomousActor::MidConeScore() {
  set_arm_goal_position(
      control_loops::superstructure::arm::ScoreFrontMidConeUpAutoIndex());
  set_wrist_goal(0.0);
  SendSuperstructureGoal();
}

void AutonomousActor::Neutral() {
  set_arm_goal_position(control_loops::superstructure::arm::NeutralIndex());
  set_wrist_goal(0.0);
  SendSuperstructureGoal();
}

void AutonomousActor::Balance() {
  set_arm_goal_position(
      control_loops::superstructure::arm::ScoreFrontLowCubeIndex());
  set_wrist_goal(0.0);
  SendSuperstructureGoal();
}

void AutonomousActor::HighCubeScore() {
  set_arm_goal_position(
      control_loops::superstructure::arm::ScoreFrontHighCubeIndex());
  set_wrist_goal(0.6);
  SendSuperstructureGoal();
}

void AutonomousActor::MidCubeScore() {
  set_arm_goal_position(
      control_loops::superstructure::arm::ScoreFrontMidCubeIndex());
  set_wrist_goal(1.0);
  SendSuperstructureGoal();
}

void AutonomousActor::PickupCube() {
  set_arm_goal_position(
      control_loops::superstructure::arm::GroundPickupBackCubeIndex());
  set_wrist_goal(1.0);
  SendSuperstructureGoal();
}

void AutonomousActor::Spit() {
  set_roller_goal(control_loops::superstructure::RollerGoal::SPIT);
  SendSuperstructureGoal();
}

void AutonomousActor::StopSpitting() {
  set_roller_goal(control_loops::superstructure::RollerGoal::IDLE);
  SendSuperstructureGoal();
}

void AutonomousActor::IntakeCube() {
  set_roller_goal(control_loops::superstructure::RollerGoal::INTAKE_CUBE);
  SendSuperstructureGoal();
}

[[nodiscard]] bool AutonomousActor::WaitForArmGoal(double distance_to_go) {
  constexpr double kEpsTheta = 0.10;

  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      ActorBase::kLoopOffset);

  bool at_goal = false;
  while (!at_goal) {
    if (ShouldCancel()) {
      return false;
    }

    phased_loop.SleepUntilNext();
    superstructure_status_fetcher_.Fetch();
    CHECK(superstructure_status_fetcher_.get() != nullptr);

    at_goal = (arm_goal_position_ ==
                   superstructure_status_fetcher_->arm()->current_node() &&
               superstructure_status_fetcher_->arm()->path_distance_to_go() <
                   distance_to_go) &&
              (std::abs(wrist_goal_ -
                        superstructure_status_fetcher_->wrist()->position()) <
               kEpsTheta);
  }

  return true;
}

}  // namespace autonomous
}  // namespace y2023
