#include "y2024/autonomous/autonomous_actor.h"

#include <chrono>
#include <cinttypes>
#include <cmath>

#include "aos/logging/logging.h"
#include "aos/util/math.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2024/autonomous/auto_splines.h"
#include "y2024/constants.h"
#include "y2024/control_loops/drivetrain/drivetrain_base.h"

DEFINE_bool(spline_auto, false, "Run simple test S-spline auto mode.");
DEFINE_bool(do_fifth_piece, true, "");

namespace y2024::autonomous {

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

AutonomousActor::AutonomousActor(::aos::EventLoop *event_loop,
                                 const y2024::Constants *robot_constants)
    : frc971::autonomous::UserButtonLocalizedAutonomousActor(
          event_loop,
          control_loops::drivetrain::GetDrivetrainConfig(event_loop)),
      localizer_control_sender_(
          event_loop->MakeSender<
              ::frc971::control_loops::drivetrain::LocalizerControl>(
              "/drivetrain")),
      superstructure_goal_sender_(
          event_loop
              ->MakeSender<::y2024::control_loops::superstructure::GoalStatic>(
                  "/superstructure")),
      superstructure_status_fetcher_(
          event_loop
              ->MakeFetcher<::y2024::control_loops::superstructure::Status>(
                  "/superstructure")),
      robot_constants_(CHECK_NOTNULL(robot_constants)),
      auto_splines_() {}

void AutonomousActor::Replan() {
  AutonomousMode mode = robot_constants_->common()->autonomous_mode();
  switch (mode) {
    case AutonomousMode::NONE:
      break;
    case AutonomousMode::SPLINE_AUTO:
      test_spline_ =
          PlanSpline(std::bind(&AutonomousSplines::TestSpline, &auto_splines_,
                               std::placeholders::_1, alliance_),
                     SplineDirection::kForward);

      starting_position_ = test_spline_->starting_position();
      break;
    case AutonomousMode::MOBILITY_AND_SHOOT:
      AOS_LOG(INFO, "Mobility & shoot replanning!");
      mobility_and_shoot_splines_ = {PlanSpline(
          std::bind(&AutonomousSplines::MobilityAndShootSpline, &auto_splines_,
                    std::placeholders::_1, alliance_),
          SplineDirection::kForward)};

      starting_position_ =
          mobility_and_shoot_splines_.value()[0].starting_position();
      CHECK(starting_position_);
      break;
    case AutonomousMode::FOUR_PIECE:
      AOS_LOG(INFO, "FOUR_PIECE replanning!");
      four_piece_splines_ = {
          PlanSpline(
              std::bind(&AutonomousSplines::FourPieceSpline1, &auto_splines_,
                        std::placeholders::_1, alliance_),
              SplineDirection::kForward),
          PlanSpline(
              std::bind(&AutonomousSplines::FourPieceSpline2, &auto_splines_,
                        std::placeholders::_1, alliance_),
              SplineDirection::kBackward),
          PlanSpline(
              std::bind(&AutonomousSplines::FourPieceSpline3, &auto_splines_,
                        std::placeholders::_1, alliance_),
              SplineDirection::kForward),
          PlanSpline(
              std::bind(&AutonomousSplines::FourPieceSpline4, &auto_splines_,
                        std::placeholders::_1, alliance_),
              SplineDirection::kForward),
          PlanSpline(
              std::bind(&AutonomousSplines::FourPieceSpline5, &auto_splines_,
                        std::placeholders::_1, alliance_),
              SplineDirection::kBackward)};

      starting_position_ = four_piece_splines_.value()[0].starting_position();
      CHECK(starting_position_);
      break;
  }

  is_planned_ = true;

  MaybeSendStartingPosition();
}

bool AutonomousActor::Run(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  AOS_LOG(INFO, "Params are %d\n", params->mode());
  if (alliance_ == aos::Alliance::kInvalid) {
    AOS_LOG(INFO, "Aborting autonomous due to invalid alliance selection.");
    return false;
  }

  AutonomousMode mode = robot_constants_->common()->autonomous_mode();
  switch (mode) {
    case AutonomousMode::NONE:
      AOS_LOG(WARNING, "No auto mode selected.");
      break;
    case AutonomousMode::SPLINE_AUTO:
      SplineAuto();
      break;
    case AutonomousMode::MOBILITY_AND_SHOOT:
      MobilityAndShoot();
      break;
    case AutonomousMode::FOUR_PIECE:
      FourPieceAuto();
      break;
  }
  return true;
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

void AutonomousActor::Reset() {
  set_intake_goal(control_loops::superstructure::IntakeGoal::NONE);
  set_note_goal(control_loops::superstructure::NoteGoal::NONE);
  set_auto_aim(false);
  set_fire(false);
  set_preloaded(false);
  SendSuperstructureGoal();
}

void AutonomousActor::SplineAuto() {
  CHECK(test_spline_);

  if (!test_spline_->WaitForPlan()) return;
  test_spline_->Start();

  if (!test_spline_->WaitForSplineDistanceRemaining(0.02)) return;
}

void AutonomousActor::MobilityAndShoot() {
  aos::monotonic_clock::time_point start_time = aos::monotonic_clock::now();

  uint32_t initial_shot_count = shot_count();

  CHECK(mobility_and_shoot_splines_);

  auto &splines = *mobility_and_shoot_splines_;

  AOS_LOG(INFO, "Going to preload");

  // Always be auto-aiming.
  Aim();

  if (!WaitForPreloaded()) return;

  AOS_LOG(INFO, "Starting to Move");

  if (!splines[0].WaitForPlan()) return;

  splines[0].Start();

  if (!splines[0].WaitForSplineDistanceRemaining(0.05)) return;

  AOS_LOG(
      INFO, "Got there %lf s\nNow Shooting\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  Shoot();

  if (!WaitForNoteFired(initial_shot_count, std::chrono::seconds(5))) return;

  StopFiring();

  AOS_LOG(
      INFO, "Note fired at %lf seconds\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));
}

void AutonomousActor::FourPieceAuto() {
  aos::monotonic_clock::time_point start_time = aos::monotonic_clock::now();

  CHECK(four_piece_splines_);
  auto &splines = *four_piece_splines_;

  uint32_t initial_shot_count = shot_count();

  // Always be aiming & firing.
  Aim();
  if (!WaitForPreloaded()) return;

  std::this_thread::sleep_for(chrono::milliseconds(500));
  Shoot();

  AOS_LOG(
      INFO, "Shooting Preloaded Note %lfs\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  if (!WaitForNoteFired(initial_shot_count, std::chrono::seconds(2))) return;

  AOS_LOG(
      INFO, "Shot first note %lfs\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  Intake();
  StopFiring();

  AOS_LOG(
      INFO, "Starting Spline 1 %lfs\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  if (!splines[0].WaitForPlan()) return;

  splines[0].Start();

  if (!splines[0].WaitForSplineDistanceRemaining(0.01)) return;

  if (!splines[1].WaitForPlan()) return;

  AOS_LOG(
      INFO, "Starting second spline %lfs\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  splines[1].Start();

  if (!splines[1].WaitForSplineDistanceRemaining(0.01)) return;

  AOS_LOG(
      INFO, "Finished second spline %lfs\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  std::this_thread::sleep_for(chrono::milliseconds(250));

  Shoot();

  if (!WaitForNoteFired(initial_shot_count + 1, std::chrono::seconds(2)))
    return;

  AOS_LOG(
      INFO, "Shot second note, starting drive %lfs\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  if (!splines[2].WaitForPlan()) return;
  splines[2].Start();

  if (!splines[2].WaitForSplineDistanceRemaining(0.01)) return;

  if (!WaitForNoteFired(initial_shot_count + 3, std::chrono::seconds(5)))
    return;

  AOS_LOG(
      INFO, "Finished 4 notes at %lfs\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  if (!FLAGS_do_fifth_piece) {
    AOS_LOG(INFO, "Exitting early due to --nodo_fifth_piece");
    return;
  }

  if (!splines[3].WaitForPlan()) return;
  splines[3].Start();

  if (!splines[3].WaitForSplineDistanceRemaining(0.1)) return;

  AOS_LOG(
      INFO, "Got to 5th note at %lfs\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  StopFiring();

  if (!splines[4].WaitForPlan()) return;
  splines[4].Start();

  if (!splines[4].WaitForSplineDistanceRemaining(0.05)) return;

  AOS_LOG(
      INFO, "Done with spline %lfs\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  std::this_thread::sleep_for(chrono::milliseconds(300));

  AOS_LOG(
      INFO, "Shooting last note! %lfs\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));

  Shoot();

  if (!WaitForNoteFired(initial_shot_count + 4, std::chrono::seconds(5)))
    return;

  AOS_LOG(
      INFO, "Done %lfs\n",
      aos::time::DurationInSeconds(aos::monotonic_clock::now() - start_time));
}

void AutonomousActor::SendSuperstructureGoal() {
  aos::Sender<control_loops::superstructure::GoalStatic>::StaticBuilder
      goal_builder = superstructure_goal_sender_.MakeStaticBuilder();

  goal_builder->set_intake_goal(intake_goal_);
  if (intake_goal_ == control_loops::superstructure::IntakeGoal::INTAKE) {
    goal_builder->set_intake_pivot(
        control_loops::superstructure::IntakePivotGoal::DOWN);
  } else {
    goal_builder->set_intake_pivot(
        control_loops::superstructure::IntakePivotGoal::UP);
  }
  goal_builder->set_note_goal(note_goal_);
  goal_builder->set_fire(fire_);
  goal_builder->set_climber_goal(
      control_loops::superstructure::ClimberGoal::STOWED);

  control_loops::superstructure::ShooterGoalStatic *shooter_goal =
      goal_builder->add_shooter_goal();

  shooter_goal->set_auto_aim(auto_aim_);
  shooter_goal->set_preloaded(preloaded_);

  goal_builder.CheckOk(goal_builder.Send());
}

void AutonomousActor::Intake() {
  set_intake_goal(control_loops::superstructure::IntakeGoal::INTAKE);
  set_note_goal(control_loops::superstructure::NoteGoal::CATAPULT);
  SendSuperstructureGoal();
}

void AutonomousActor::Aim() {
  set_auto_aim(true);
  SendSuperstructureGoal();
}

void AutonomousActor::Shoot() {
  set_fire(true);
  SendSuperstructureGoal();
}

void AutonomousActor::StopFiring() {
  set_fire(false);
  SendSuperstructureGoal();
}

[[nodiscard]] bool AutonomousActor::WaitForPreloaded() {
  set_preloaded(true);
  SendSuperstructureGoal();

  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      aos::common::actions::kLoopOffset);

  bool loaded = false;
  while (!loaded) {
    if (ShouldCancel()) {
      return false;
    }

    phased_loop.SleepUntilNext();
    superstructure_status_fetcher_.Fetch();
    CHECK(superstructure_status_fetcher_.get() != nullptr);

    loaded = (superstructure_status_fetcher_->shooter()->catapult_state() ==
              control_loops::superstructure::CatapultState::LOADED);
  }

  set_preloaded(false);
  SendSuperstructureGoal();

  return true;
}

uint32_t AutonomousActor::shot_count() {
  superstructure_status_fetcher_.Fetch();
  return superstructure_status_fetcher_->shot_count();
}

[[nodiscard]] bool AutonomousActor::WaitForNoteFired(
    uint32_t penultimate_target_shot_count, std::chrono::nanoseconds timeout) {
  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      aos::common::actions::kLoopOffset);
  aos::monotonic_clock::time_point end_time =
      event_loop()->monotonic_now() + timeout;
  while (true) {
    if (ShouldCancel()) {
      return false;
    }

    phased_loop.SleepUntilNext();

    if (shot_count() > penultimate_target_shot_count ||
        event_loop()->monotonic_now() > end_time) {
      return true;
    }
  }
}

[[nodiscard]] bool AutonomousActor::WaitForCatapultReady() {
  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      aos::common::actions::kLoopOffset);

  bool loaded = false;
  while (!loaded) {
    if (ShouldCancel()) {
      return false;
    }

    phased_loop.SleepUntilNext();
    superstructure_status_fetcher_.Fetch();
    CHECK(superstructure_status_fetcher_.get() != nullptr);

    loaded = (superstructure_status_fetcher_->state() ==
              control_loops::superstructure::SuperstructureState::READY);
  }

  SendSuperstructureGoal();

  return true;
}

}  // namespace y2024::autonomous
