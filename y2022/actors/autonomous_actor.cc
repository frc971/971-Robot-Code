#include "y2022/actors/autonomous_actor.h"

#include <chrono>
#include <cinttypes>
#include <cmath>

#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "aos/util/math.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "y2022/actors/auto_splines.h"
#include "y2022/constants.h"
#include "y2022/control_loops/drivetrain/drivetrain_base.h"

DEFINE_bool(spline_auto, false, "If true, define a spline autonomous mode");
DEFINE_bool(rapid_react, false,
            "If true, run the main rapid react autonomous mode");

namespace y2022 {
namespace actors {
namespace {
constexpr double kExtendIntakeGoal = 0.0;
constexpr double kRetractIntakeGoal = 1.47;
constexpr double kRollerVoltage = 12.0;
constexpr double kCatapultReturnPosition = -0.908;
}  // namespace

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
      superstructure_goal_sender_(
          event_loop->MakeSender<control_loops::superstructure::Goal>(
              "/superstructure")),
      superstructure_status_fetcher_(
          event_loop->MakeFetcher<control_loops::superstructure::Status>(
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

void AutonomousActor::Replan() {
  LOG(INFO) << "Alliance " << static_cast<int>(alliance_);
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
  } else if (FLAGS_rapid_react) {
    rapid_react_splines_ = {
        PlanSpline(std::bind(&AutonomousSplines::Spline1, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kForward),
        PlanSpline(std::bind(&AutonomousSplines::Spline2, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kForward),
        PlanSpline(std::bind(&AutonomousSplines::Spline3, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kForward),
        PlanSpline(std::bind(&AutonomousSplines::Spline4, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kBackward),
        PlanSpline(std::bind(&AutonomousSplines::Spline5, &auto_splines_,
                             std::placeholders::_1, alliance_),
                   SplineDirection::kBackward)};
    starting_position_ = rapid_react_splines_.value()[0].starting_position();
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
  RetractFrontIntake();
  RetractBackIntake();

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
  } else if (FLAGS_rapid_react) {
    RapidReact();
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
  if (builder.Send(localizer_control_builder.Finish()) !=
      aos::RawSender::Error::kOk) {
    AOS_LOG(ERROR, "Failed to reset localizer.\n");
  }
}

void AutonomousActor::SplineAuto() {
  CHECK(test_spline_);

  if (!test_spline_->WaitForPlan()) return;
  test_spline_->Start();

  if (!test_spline_->WaitForSplineDistanceRemaining(0.02)) return;
}

void AutonomousActor::RapidReact() {
  aos::monotonic_clock::time_point start_time = aos::monotonic_clock::now();

  CHECK(rapid_react_splines_);

  auto &splines = *rapid_react_splines_;

  // Tell the superstructure a ball was preloaded

  if (!WaitForPreloaded()) return;
  // Drive and intake the 2nd ball
  ExtendFrontIntake();
  if (!splines[0].WaitForPlan()) return;
  splines[0].Start();
  if (!splines[0].WaitForSplineDistanceRemaining(0.02)) return;

  // Fire the two balls once we stopped
  set_fire_at_will(true);
  SendSuperstructureGoal();
  if (!WaitForBallsShot(2)) return;
  set_fire_at_will(false);

  // Drive and intake the 3rd ball
  if (!splines[1].WaitForPlan()) return;
  splines[1].Start();
  if (!splines[1].WaitForSplineDistanceRemaining(0.02)) return;

  // Fire the 3rd once we stopped.
  set_fire_at_will(true);
  SendSuperstructureGoal();
  if (!WaitForBallsShot(1)) return;
  set_fire_at_will(false);

  // Drive to the human player station while intaking two balls.
  // Once is already placed down,
  // and one will be rolled to the robot by the human player
  if (!splines[2].WaitForPlan()) return;
  splines[2].Start();
  if (!splines[2].WaitForSplineDistanceRemaining(0.02)) return;

  // Drive to the shooting position
  if (!splines[3].WaitForPlan()) return;
  splines[3].Start();
  if (!splines[3].WaitForSplineDistanceRemaining(0.02)) return;

  // Fire the two balls once we stopped
  set_fire_at_will(true);
  SendSuperstructureGoal();
  if (!WaitForBallsShot(2)) return;
  set_fire_at_will(false);

  // Done intaking
  RetractFrontIntake();

  // Drive to the middle of the field to get ready for teleop
  if (!splines[4].WaitForPlan()) return;
  splines[4].Start();
  if (!splines[4].WaitForSplineDistanceRemaining(0.02)) return;

  LOG(INFO) << "Took "
            << chrono::duration<double>(aos::monotonic_clock::now() -
                                        start_time)
                   .count()
            << 's';
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

    loaded = (superstructure_status_fetcher_->state() ==
              control_loops::superstructure::SuperstructureState::LOADED);
  }

  set_preloaded(false);
  SendSuperstructureGoal();

  return true;
}

void AutonomousActor::SendSuperstructureGoal() {
  auto builder = superstructure_goal_sender_.MakeBuilder();

  flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
      intake_front_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *builder.fbb(), intake_front_goal_,
          CreateProfileParameters(*builder.fbb(), 20.0, 60.0));

  flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
      intake_back_offset = CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
          *builder.fbb(), intake_back_goal_,
          CreateProfileParameters(*builder.fbb(), 20.0, 60.0));

  flatbuffers::Offset<StaticZeroingSingleDOFProfiledSubsystemGoal>
      catapult_return_position_offset =
          CreateStaticZeroingSingleDOFProfiledSubsystemGoal(
              *builder.fbb(), kCatapultReturnPosition,
              CreateProfileParameters(*builder.fbb(), 9.0, 50.0));

  superstructure::CatapultGoal::Builder catapult_goal_builder(*builder.fbb());
  catapult_goal_builder.add_shot_position(0.03);
  catapult_goal_builder.add_shot_velocity(18.0);
  catapult_goal_builder.add_return_position(catapult_return_position_offset);
  flatbuffers::Offset<superstructure::CatapultGoal> catapult_goal_offset =
      catapult_goal_builder.Finish();

  superstructure::Goal::Builder superstructure_builder =
      builder.MakeBuilder<superstructure::Goal>();

  superstructure_builder.add_intake_front(intake_front_offset);
  superstructure_builder.add_intake_back(intake_back_offset);
  superstructure_builder.add_roller_speed_compensation(1.5);
  superstructure_builder.add_roller_speed_front(roller_front_voltage_);
  superstructure_builder.add_roller_speed_back(roller_back_voltage_);
  superstructure_builder.add_transfer_roller_speed(transfer_roller_voltage_);
  superstructure_builder.add_catapult(catapult_goal_offset);
  superstructure_builder.add_fire(fire_);
  superstructure_builder.add_preloaded(preloaded_);
  superstructure_builder.add_auto_aim(true);

  if (builder.Send(superstructure_builder.Finish()) !=
      aos::RawSender::Error::kOk) {
    AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
  }
}

void AutonomousActor::ExtendFrontIntake() {
  set_intake_front_goal(kExtendIntakeGoal);
  set_roller_front_voltage(kRollerVoltage);
  set_transfer_roller_voltage(kRollerVoltage);
  SendSuperstructureGoal();
}

void AutonomousActor::RetractFrontIntake() {
  set_intake_front_goal(kRetractIntakeGoal);
  set_roller_front_voltage(kRollerVoltage);
  set_transfer_roller_voltage(0.0);
  SendSuperstructureGoal();
}

void AutonomousActor::ExtendBackIntake() {
  set_intake_back_goal(kExtendIntakeGoal);
  set_roller_back_voltage(kRollerVoltage);
  set_transfer_roller_voltage(-kRollerVoltage);
  SendSuperstructureGoal();
}

void AutonomousActor::RetractBackIntake() {
  set_intake_back_goal(kRetractIntakeGoal);
  set_roller_back_voltage(kRollerVoltage);
  set_transfer_roller_voltage(0.0);
  SendSuperstructureGoal();
}

[[nodiscard]] bool AutonomousActor::WaitForBallsShot(int num_wanted) {
  ::aos::time::PhasedLoop phased_loop(frc971::controls::kLoopFrequency,
                                      event_loop()->monotonic_now(),
                                      ActorBase::kLoopOffset);
  superstructure_status_fetcher_.Fetch();
  CHECK(superstructure_status_fetcher_.get() != nullptr);
  int initial_balls = superstructure_status_fetcher_->shot_count();
  LOG(INFO) << "Waiting for balls, started with " << initial_balls;
  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    superstructure_status_fetcher_.Fetch();
    CHECK(superstructure_status_fetcher_.get() != nullptr);
    if (superstructure_status_fetcher_->shot_count() - initial_balls >=
        num_wanted) {
      return true;
    }
  }
}

}  // namespace actors
}  // namespace y2022
