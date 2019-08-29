#include "y2016/actors/autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/logging/logging.h"
#include "aos/util/phased_loop.h"

#include "y2016/control_loops/drivetrain/drivetrain_base.h"
#include "y2016/control_loops/shooter/shooter_goal_generated.h"
#include "y2016/control_loops/shooter/shooter_status_generated.h"
#include "y2016/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2016/control_loops/superstructure/superstructure_status_generated.h"
#include "y2016/queues/ball_detector_generated.h"
#include "y2016/vision/vision_generated.h"

namespace y2016 {
namespace actors {
using ::aos::monotonic_clock;
using ::frc971::ProfileParametersT;
namespace superstructure = y2016::control_loops::superstructure;
namespace shooter = y2016::control_loops::shooter;
namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;

namespace {
ProfileParametersT MakeProfileParameters(float max_velocity,
                                         float max_acceleration) {
  ProfileParametersT result;
  result.max_velocity = max_velocity;
  result.max_acceleration = max_acceleration;
  return result;
}

const ProfileParametersT kSlowDrive = MakeProfileParameters(0.8, 2.5);
const ProfileParametersT kLowBarDrive = MakeProfileParameters(1.3, 2.5);
const ProfileParametersT kMoatDrive = MakeProfileParameters(1.2, 3.5);
const ProfileParametersT kRealignDrive = MakeProfileParameters(2.0, 2.5);
const ProfileParametersT kRockWallDrive = MakeProfileParameters(0.8, 2.5);
const ProfileParametersT kFastDrive = MakeProfileParameters(3.0, 2.5);

const ProfileParametersT kSlowTurn = MakeProfileParameters(0.8, 3.0);
const ProfileParametersT kFastTurn = MakeProfileParameters(3.0, 10.0);
const ProfileParametersT kStealTurn = MakeProfileParameters(4.0, 15.0);
const ProfileParametersT kSwerveTurn = MakeProfileParameters(2.0, 7.0);
const ProfileParametersT kFinishTurn = MakeProfileParameters(2.0, 5.0);

const ProfileParametersT kTwoBallLowDrive = MakeProfileParameters(1.7, 3.5);
const ProfileParametersT kTwoBallFastDrive = MakeProfileParameters(3.0, 1.5);
const ProfileParametersT kTwoBallReturnDrive = MakeProfileParameters(3.0, 1.9);
const ProfileParametersT kTwoBallReturnSlow = MakeProfileParameters(3.0, 2.5);
const ProfileParametersT kTwoBallBallPickup = MakeProfileParameters(2.0, 1.75);
const ProfileParametersT kTwoBallBallPickupAccel =
    MakeProfileParameters(2.0, 2.5);

}  // namespace

AutonomousActor::AutonomousActor(::aos::EventLoop *event_loop)
    : frc971::autonomous::BaseAutonomousActor(
          event_loop, control_loops::drivetrain::GetDrivetrainConfig()),
      vision_align_actor_factory_(
          actors::VisionAlignActor::MakeFactory(event_loop)),
      vision_status_fetcher_(
          event_loop->MakeFetcher<::y2016::vision::VisionStatus>(
              "/superstructure")),
      ball_detector_fetcher_(
          event_loop->MakeFetcher<::y2016::sensors::BallDetector>(
              "/superstructure")),
      shooter_goal_sender_(
          event_loop->MakeSender<::y2016::control_loops::shooter::Goal>(
              "/shooter")),
      shooter_status_fetcher_(
          event_loop->MakeFetcher<::y2016::control_loops::shooter::Status>(
              "/shooter")),
      superstructure_status_fetcher_(
          event_loop
              ->MakeFetcher<::y2016::control_loops::superstructure::Status>(
                  "/superstructure")),
      superstructure_goal_sender_(
          event_loop->MakeSender<::y2016::control_loops::superstructure::Goal>(
              "/superstructure")) {}

constexpr double kDoNotTurnCare = 2.0;

void AutonomousActor::MoveSuperstructure(
    double intake, double shoulder, double wrist,
    const ProfileParametersT intake_params,
    const ProfileParametersT shoulder_params,
    const ProfileParametersT wrist_params, bool traverse_up,
    double roller_power) {
  superstructure_goal_ = {intake, shoulder, wrist};

  auto builder = superstructure_goal_sender_.MakeBuilder();

  superstructure::Goal::Builder superstructure_goal_builder =
      builder.MakeBuilder<superstructure::Goal>();

  superstructure_goal_builder.add_angle_intake(intake);
  superstructure_goal_builder.add_angle_shoulder(shoulder);
  superstructure_goal_builder.add_angle_wrist(wrist);

  superstructure_goal_builder.add_max_angular_velocity_intake(
      intake_params.max_velocity);
  superstructure_goal_builder.add_max_angular_velocity_shoulder(
      shoulder_params.max_velocity);
  superstructure_goal_builder.add_max_angular_velocity_wrist(
      wrist_params.max_velocity);

  superstructure_goal_builder.add_max_angular_acceleration_intake(
      intake_params.max_acceleration);
  superstructure_goal_builder.add_max_angular_acceleration_shoulder(
      shoulder_params.max_acceleration);
  superstructure_goal_builder.add_max_angular_acceleration_wrist(
      wrist_params.max_acceleration);

  superstructure_goal_builder.add_voltage_top_rollers(roller_power);
  superstructure_goal_builder.add_voltage_bottom_rollers(roller_power);

  superstructure_goal_builder.add_traverse_unlatched(true);
  superstructure_goal_builder.add_traverse_down(!traverse_up);
  superstructure_goal_builder.add_voltage_climber(0.0);
  superstructure_goal_builder.add_unfold_climber(false);

  if (!builder.Send(superstructure_goal_builder.Finish())) {
    AOS_LOG(ERROR, "Sending superstructure goal failed.\n");
  }
}

void AutonomousActor::OpenShooter() {
  shooter_speed_ = 0.0;

  auto builder = shooter_goal_sender_.MakeBuilder();
  shooter::Goal::Builder shooter_goal_builder =
      builder.MakeBuilder<shooter::Goal>();
  shooter_goal_builder.add_angular_velocity(shooter_speed_);
  shooter_goal_builder.add_clamp_open(true);
  shooter_goal_builder.add_push_to_shooter(false);
  shooter_goal_builder.add_force_lights_on(false);
  if (!builder.Send(shooter_goal_builder.Finish())) {
    AOS_LOG(ERROR, "Sending shooter goal failed.\n");
  }
}

void AutonomousActor::CloseShooter() {
  shooter_speed_ = 0.0;

  auto builder = shooter_goal_sender_.MakeBuilder();
  shooter::Goal::Builder shooter_goal_builder =
      builder.MakeBuilder<shooter::Goal>();
  shooter_goal_builder.add_angular_velocity(shooter_speed_);
  shooter_goal_builder.add_clamp_open(false);
  shooter_goal_builder.add_push_to_shooter(false);
  shooter_goal_builder.add_force_lights_on(false);

  if (!builder.Send(shooter_goal_builder.Finish())) {
    AOS_LOG(ERROR, "Sending shooter goal failed.\n");
  }
}

void AutonomousActor::SetShooterSpeed(double speed) {
  shooter_speed_ = speed;

  // In auto, we want to have the lights on whenever possible since we have no
  // hope of a human aligning the robot.
  bool force_lights_on = shooter_speed_ > 1.0;

  auto builder = shooter_goal_sender_.MakeBuilder();
  shooter::Goal::Builder shooter_goal_builder =
      builder.MakeBuilder<shooter::Goal>();
  shooter_goal_builder.add_angular_velocity(shooter_speed_);
  shooter_goal_builder.add_clamp_open(false);
  shooter_goal_builder.add_push_to_shooter(false);
  shooter_goal_builder.add_force_lights_on(force_lights_on);

  if (!builder.Send(shooter_goal_builder.Finish())) {
    AOS_LOG(ERROR, "Sending shooter goal failed.\n");
  }
}

void AutonomousActor::Shoot() {
  uint32_t initial_shots = 0;

  shooter_status_fetcher_.Fetch();
  if (shooter_status_fetcher_.get()) {
    initial_shots = shooter_status_fetcher_->shots();
  }

  // In auto, we want to have the lights on whenever possible since we have no
  // hope of a human aligning the robot.
  bool force_lights_on = shooter_speed_ > 1.0;

  auto builder = shooter_goal_sender_.MakeBuilder();
  shooter::Goal::Builder shooter_goal_builder =
      builder.MakeBuilder<shooter::Goal>();
  shooter_goal_builder.add_angular_velocity(shooter_speed_);
  shooter_goal_builder.add_clamp_open(false);
  shooter_goal_builder.add_push_to_shooter(true);
  shooter_goal_builder.add_force_lights_on(force_lights_on);

  if (!builder.Send(shooter_goal_builder.Finish())) {
    AOS_LOG(ERROR, "Sending shooter goal failed.\n");
  }

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      event_loop()->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) return;

    // Wait for the shot count to change so we know when the shot is complete.
    shooter_status_fetcher_.Fetch();
    if (shooter_status_fetcher_.get()) {
      if (initial_shots < shooter_status_fetcher_->shots()) {
        return;
      }
    }
    phased_loop.SleepUntilNext();
  }
}

void AutonomousActor::WaitForShooterSpeed() {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      event_loop()->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) return;

    shooter_status_fetcher_.Fetch();
    if (shooter_status_fetcher_.get()) {
      if (shooter_status_fetcher_->left()->ready() &&
          shooter_status_fetcher_->right()->ready()) {
        return;
      }
    }
    phased_loop.SleepUntilNext();
  }
}

void AutonomousActor::AlignWithVisionGoal() {
  vision_align_action::VisionAlignActionParamsT params;
  vision_action_ = vision_align_actor_factory_.Make(params);
  vision_action_->Start();
}

void AutonomousActor::WaitForAlignedWithVision(
    chrono::nanoseconds align_duration) {
  bool vision_valid = false;
  double last_angle = 0.0;
  int ready_to_fire = 0;

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      event_loop()->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);
  const monotonic_clock::time_point end_time = monotonic_now() + align_duration;
  while (end_time > monotonic_now()) {
    if (ShouldCancel()) break;

    vision_status_fetcher_.Fetch();
    if (vision_status_fetcher_.get()) {
      vision_valid = (vision_status_fetcher_->left_image_valid() &&
                      vision_status_fetcher_->right_image_valid());
      last_angle = vision_status_fetcher_->horizontal_angle();
    }

    drivetrain_status_fetcher_.Fetch();
    drivetrain_goal_fetcher_.Fetch();

    if (drivetrain_status_fetcher_.get() && drivetrain_goal_fetcher_.get()) {
      const double left_goal = drivetrain_goal_fetcher_->left_goal();
      const double right_goal = drivetrain_goal_fetcher_->right_goal();
      const double left_current =
          drivetrain_status_fetcher_->estimated_left_position();
      const double right_current =
          drivetrain_status_fetcher_->estimated_right_position();
      const double left_velocity =
          drivetrain_status_fetcher_->estimated_left_velocity();
      const double right_velocity =
          drivetrain_status_fetcher_->estimated_right_velocity();

      if (vision_valid && ::std::abs(last_angle) < 0.02 &&
          ::std::abs((left_goal - right_goal) -
                     (left_current - right_current)) /
                  dt_config_.robot_radius / 2.0 <
              0.02 &&
          ::std::abs(left_velocity - right_velocity) < 0.01) {
        ++ready_to_fire;
      } else {
        ready_to_fire = 0;
      }
      if (ready_to_fire > 15) {
        break;
        AOS_LOG(INFO, "Vision align success!\n");
      }
    }
    phased_loop.SleepUntilNext();
  }

  vision_action_->Cancel();
  WaitUntilDoneOrCanceled(::std::move(vision_action_));
  AOS_LOG(INFO, "Done waiting for vision\n");
}

bool AutonomousActor::IntakeDone() {
  superstructure_status_fetcher_.Fetch();

  constexpr double kProfileError = 1e-5;
  constexpr double kEpsilon = 0.15;

  if (superstructure_status_fetcher_->state() < 12 ||
      superstructure_status_fetcher_->state() == 16) {
    AOS_LOG(ERROR, "Superstructure no longer running, aborting action\n");
    return true;
  }

  if (::std::abs(superstructure_status_fetcher_->intake()->goal_angle() -
                 superstructure_goal_.intake) < kProfileError &&
      ::std::abs(
          superstructure_status_fetcher_->intake()->goal_angular_velocity()) <
          kProfileError) {
    AOS_LOG(DEBUG, "Profile done.\n");
    if (::std::abs(superstructure_status_fetcher_->intake()->angle() -
                   superstructure_goal_.intake) < kEpsilon &&
        ::std::abs(
            superstructure_status_fetcher_->intake()->angular_velocity()) <
            kEpsilon) {
      AOS_LOG(INFO, "Near goal, done.\n");
      return true;
    }
  }
  return false;
}

bool AutonomousActor::SuperstructureProfileDone() {
  if (superstructure_status_fetcher_->state() < 12 ||
      superstructure_status_fetcher_->state() == 16) {
    AOS_LOG(ERROR, "Superstructure no longer running, aborting action\n");
    return true;
  }

  constexpr double kProfileError = 1e-5;
  return ::std::abs(superstructure_status_fetcher_->intake()->goal_angle() -
                    superstructure_goal_.intake) < kProfileError &&
         ::std::abs(superstructure_status_fetcher_->shoulder()->goal_angle() -
                    superstructure_goal_.shoulder) < kProfileError &&
         ::std::abs(superstructure_status_fetcher_->wrist()->goal_angle() -
                    superstructure_goal_.wrist) < kProfileError &&
         ::std::abs(superstructure_status_fetcher_->intake()
                        ->goal_angular_velocity()) < kProfileError &&
         ::std::abs(superstructure_status_fetcher_->shoulder()
                        ->goal_angular_velocity()) < kProfileError &&
         ::std::abs(
             superstructure_status_fetcher_->wrist()->goal_angular_velocity()) <
             kProfileError;
}

bool AutonomousActor::SuperstructureDone() {
  superstructure_status_fetcher_.Fetch();

  constexpr double kEpsilon = 0.03;
  if (SuperstructureProfileDone()) {
    AOS_LOG(DEBUG, "Profile done.\n");
    if (::std::abs(superstructure_status_fetcher_->intake()->angle() -
                   superstructure_goal_.intake) < (kEpsilon + 0.1) &&
        ::std::abs(superstructure_status_fetcher_->shoulder()->angle() -
                   superstructure_goal_.shoulder) < (kEpsilon + 0.05) &&
        ::std::abs(superstructure_status_fetcher_->wrist()->angle() -
                   superstructure_goal_.wrist) < (kEpsilon + 0.01) &&
        ::std::abs(
            superstructure_status_fetcher_->intake()->angular_velocity()) <
            (kEpsilon + 0.1) &&
        ::std::abs(
            superstructure_status_fetcher_->shoulder()->angular_velocity()) <
            (kEpsilon + 0.10) &&
        ::std::abs(
            superstructure_status_fetcher_->wrist()->angular_velocity()) <
            (kEpsilon + 0.05)) {
      AOS_LOG(INFO, "Near goal, done.\n");
      return true;
    }
  }
  return false;
}

void AutonomousActor::WaitForIntake() {
  WaitUntil(::std::bind(&AutonomousActor::IntakeDone, this));
}

void AutonomousActor::WaitForSuperstructure() {
  WaitUntil(::std::bind(&AutonomousActor::SuperstructureDone, this));
}

void AutonomousActor::WaitForSuperstructureProfile() {
  WaitUntil([this]() {
    superstructure_status_fetcher_.Fetch();
    return SuperstructureProfileDone();
  });
}

void AutonomousActor::WaitForSuperstructureLow() {
  WaitUntil([this]() {
    superstructure_status_fetcher_.Fetch();

    return SuperstructureProfileDone() ||
           superstructure_status_fetcher_->shoulder()->angle() < 0.1;
  });
}

void AutonomousActor::BackLongShotLowBarTwoBall() {
  AOS_LOG(INFO, "Expanding for back long shot\n");
  MoveSuperstructure(0.00, M_PI / 2.0 - 0.2, -0.55,
                     MakeProfileParameters(7.0, 40.0),
                     MakeProfileParameters(4.0, 6.0),
                     MakeProfileParameters(10.0, 25.0), false, 0.0);
}

void AutonomousActor::BackLongShotTwoBall() {
  AOS_LOG(INFO, "Expanding for back long shot\n");
  MoveSuperstructure(0.00, M_PI / 2.0 - 0.2, -0.55,
                     MakeProfileParameters(7.0, 40.0),
                     MakeProfileParameters(4.0, 6.0),
                     MakeProfileParameters(10.0, 25.0), false, 0.0);
}

void AutonomousActor::BackLongShotTwoBallFinish() {
  AOS_LOG(INFO, "Expanding for back long shot\n");
  MoveSuperstructure(0.00, M_PI / 2.0 - 0.2, -0.625 + 0.03,
                     MakeProfileParameters(7.0, 40.0),
                     MakeProfileParameters(4.0, 6.0),
                     MakeProfileParameters(10.0, 25.0), false, 0.0);
}

void AutonomousActor::BackLongShot() {
  AOS_LOG(INFO, "Expanding for back long shot\n");
  MoveSuperstructure(0.80, M_PI / 2.0 - 0.2, -0.62,
                     MakeProfileParameters(7.0, 40.0),
                     MakeProfileParameters(4.0, 6.0),
                     MakeProfileParameters(10.0, 25.0), false, 0.0);
}

void AutonomousActor::BackMiddleShot() {
  AOS_LOG(INFO, "Expanding for back middle shot\n");
  MoveSuperstructure(-0.05, M_PI / 2.0 - 0.2, -0.665,
                     MakeProfileParameters(7.0, 40.0),
                     MakeProfileParameters(4.0, 10.0),
                     MakeProfileParameters(10.0, 25.0), false, 0.0);
}

void AutonomousActor::FrontLongShot() {
  AOS_LOG(INFO, "Expanding for front long shot\n");
  MoveSuperstructure(0.80, M_PI / 2.0 + 0.1, M_PI + 0.41 + 0.02,
                     MakeProfileParameters(7.0, 40.0),
                     MakeProfileParameters(4.0, 6.0),
                     MakeProfileParameters(10.0, 25.0), false, 0.0);
}

void AutonomousActor::FrontMiddleShot() {
  AOS_LOG(INFO, "Expanding for front middle shot\n");
  MoveSuperstructure(-0.05, M_PI / 2.0 + 0.1, M_PI + 0.44,
                     MakeProfileParameters(7.0, 40.0),
                     MakeProfileParameters(4.0, 10.0),
                     MakeProfileParameters(10.0, 25.0), true, 0.0);
}

void AutonomousActor::TuckArm(bool low_bar, bool traverse_down) {
  MoveSuperstructure(low_bar ? -0.05 : 2.0, -0.010, 0.0,
                     MakeProfileParameters(7.0, 40.0),
                     MakeProfileParameters(4.0, 10.0),
                     MakeProfileParameters(10.0, 25.0), !traverse_down, 0.0);
}

void AutonomousActor::DoFullShot() {
  if (ShouldCancel()) return;
  // Make sure that the base is aligned with the base.
  AOS_LOG(INFO, "Waiting for the superstructure\n");
  WaitForSuperstructure();

  this_thread::sleep_for(chrono::milliseconds(500));

  if (ShouldCancel()) return;
  AOS_LOG(INFO, "Triggering the vision actor\n");
  AlignWithVisionGoal();

  // Wait for the drive base to be aligned with the target and make sure that
  // the shooter is up to speed.
  AOS_LOG(INFO, "Waiting for vision to be aligned\n");
  WaitForAlignedWithVision(chrono::milliseconds(2000));
  if (ShouldCancel()) return;
  AOS_LOG(INFO, "Waiting for shooter to be up to speed\n");
  WaitForShooterSpeed();
  if (ShouldCancel()) return;

  this_thread::sleep_for(chrono::milliseconds(300));
  AOS_LOG(INFO, "Shoot!\n");
  Shoot();

  // Turn off the shooter and fold up the superstructure.
  if (ShouldCancel()) return;
  AOS_LOG(INFO, "Stopping shooter\n");
  SetShooterSpeed(0.0);
  AOS_LOG(INFO, "Folding superstructure back down\n");
  TuckArm(false, false);

  // Wait for everything to be folded up.
  AOS_LOG(INFO, "Waiting for superstructure to be folded back down\n");
  WaitForSuperstructureLow();
}

void AutonomousActor::LowBarDrive() {
  TuckArm(false, true);
  StartDrive(-5.5, 0.0, kLowBarDrive, kSlowTurn);

  if (!WaitForDriveNear(5.3, 0.0)) return;
  TuckArm(true, true);

  if (!WaitForDriveNear(5.0, 0.0)) return;

  StartDrive(0.0, 0.0, kLowBarDrive, kSlowTurn);

  if (!WaitForDriveNear(3.0, 0.0)) return;

  StartDrive(0.0, 0.0, kLowBarDrive, kSlowTurn);

  if (!WaitForDriveNear(1.0, 0.0)) return;

  StartDrive(0, -M_PI / 4.0 - 0.2, kLowBarDrive, kSlowTurn);
}

void AutonomousActor::TippyDrive(double goal_distance, double tip_distance,
                                 double below, double above) {
  StartDrive(goal_distance, 0.0, kMoatDrive, kSlowTurn);
  if (!WaitForBelowAngle(below)) return;
  if (!WaitForAboveAngle(above)) return;
  // Ok, we are good now.  Compensate by moving the goal by the error.
  // Should be here at 2.7
  drivetrain_status_fetcher_.Fetch();
  if (drivetrain_status_fetcher_.get()) {
    const double left_error =
        (initial_drivetrain_.left -
         drivetrain_status_fetcher_->estimated_left_position());
    const double right_error =
        (initial_drivetrain_.right -
         drivetrain_status_fetcher_->estimated_right_position());
    const double distance_to_go = (left_error + right_error) / 2.0;
    const double distance_compensation =
        goal_distance - tip_distance - distance_to_go;
    AOS_LOG(INFO, "Going %f further at the bump\n", distance_compensation);
    StartDrive(distance_compensation, 0.0, kMoatDrive, kSlowTurn);
  }
}

void AutonomousActor::MiddleDrive() {
  TuckArm(false, false);
  TippyDrive(3.65, 2.7, -0.2, 0.0);
  if (!WaitForDriveDone()) return;
}

void AutonomousActor::OneFromMiddleDrive(bool left) {
  const double kTurnAngle = left ? -0.41 : 0.41;
  TuckArm(false, false);
  TippyDrive(4.05, 2.7, -0.2, 0.0);

  if (!WaitForDriveDone()) return;
  StartDrive(0.0, kTurnAngle, kRealignDrive, kFastTurn);
}

void AutonomousActor::TwoFromMiddleDrive() {
  TuckArm(false, false);
  constexpr double kDriveDistance = 5.10;
  TippyDrive(kDriveDistance, 2.7, -0.2, 0.0);

  if (!WaitForDriveNear(kDriveDistance - 3.0, 2.0)) return;
  StartDrive(0, -M_PI / 2 - 0.10, kMoatDrive, kFastTurn);

  if (!WaitForDriveDone()) return;
  StartDrive(0, M_PI / 3 + 0.35, kMoatDrive, kFastTurn);
}

void AutonomousActor::CloseIfBall() {
  ball_detector_fetcher_.Fetch();
  if (ball_detector_fetcher_.get()) {
    const bool ball_detected = ball_detector_fetcher_->voltage() > 2.5;
    if (ball_detected) {
      CloseShooter();
    }
  }
}

void AutonomousActor::WaitForBallOrDriveDone() {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      event_loop()->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) {
      return;
    }
    phased_loop.SleepUntilNext();
    drivetrain_status_fetcher_.Fetch();
    if (IsDriveDone()) {
      return;
    }

    ball_detector_fetcher_.Fetch();
    if (ball_detector_fetcher_.get()) {
      const bool ball_detected = ball_detector_fetcher_->voltage() > 2.5;
      if (ball_detected) {
        return;
      }
    }
  }
}

void AutonomousActor::TwoBallAuto() {
  const monotonic_clock::time_point start_time = monotonic_now();
  OpenShooter();
  MoveSuperstructure(0.10, -0.010, 0.0, MakeProfileParameters(8.0, 60.0),
                     MakeProfileParameters(4.0, 10.0),
                     MakeProfileParameters(10.0, 25.0), false, 12.0);
  if (ShouldCancel()) return;
  AOS_LOG(INFO, "Waiting for the intake to come down.\n");

  WaitForIntake();
  AOS_LOG(INFO, "Intake done at %f seconds, starting to drive\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  if (ShouldCancel()) return;
  const double kDriveDistance = 5.05;
  StartDrive(-kDriveDistance, 0.0, kTwoBallLowDrive, kSlowTurn);

  StartDrive(0.0, 0.4, kTwoBallLowDrive, kSwerveTurn);
  if (!WaitForDriveNear(kDriveDistance - 0.5, kDoNotTurnCare)) return;

  // Check if the ball is there.
  bool first_ball_there = true;
  ball_detector_fetcher_.Fetch();
  if (ball_detector_fetcher_.get()) {
    const bool ball_detected = ball_detector_fetcher_->voltage() > 2.5;
    first_ball_there = ball_detected;
    AOS_LOG(INFO, "Saw the ball: %d at %f\n", first_ball_there,
            ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  }
  MoveSuperstructure(0.10, -0.010, 0.0, MakeProfileParameters(8.0, 40.0),
                     MakeProfileParameters(4.0, 10.0),
                     MakeProfileParameters(10.0, 25.0), false, 0.0);
  AOS_LOG(INFO,
          "Shutting off rollers at %f seconds, starting to straighten out\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  StartDrive(0.0, -0.4, kTwoBallLowDrive, kSwerveTurn);
  MoveSuperstructure(-0.05, -0.010, 0.0, MakeProfileParameters(8.0, 40.0),
                     MakeProfileParameters(4.0, 10.0),
                     MakeProfileParameters(10.0, 25.0), false, 0.0);
  CloseShooter();
  if (!WaitForDriveNear(kDriveDistance - 2.4, kDoNotTurnCare)) return;

  // We are now under the low bar.  Start lifting.
  BackLongShotLowBarTwoBall();
  AOS_LOG(INFO, "Spinning up the shooter wheels\n");
  SetShooterSpeed(640.0);
  StartDrive(0.0, 0.0, kTwoBallFastDrive, kSwerveTurn);

  if (!WaitForDriveNear(1.50, kDoNotTurnCare)) return;
  constexpr double kShootTurnAngle = -M_PI / 4.0 - 0.05;
  StartDrive(0, kShootTurnAngle, kTwoBallFastDrive, kFinishTurn);
  BackLongShotTwoBall();

  if (!WaitForDriveDone()) return;
  AOS_LOG(INFO, "First shot done driving at %f seconds\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));

  WaitForSuperstructureProfile();

  if (ShouldCancel()) return;
  AlignWithVisionGoal();

  WaitForShooterSpeed();
  if (ShouldCancel()) return;

  constexpr chrono::milliseconds kVisionExtra{0};
  WaitForAlignedWithVision(chrono::milliseconds(500) + kVisionExtra);
  BackLongShotTwoBallFinish();
  WaitForSuperstructureProfile();
  if (ShouldCancel()) return;
  AOS_LOG(INFO, "Shoot!\n");
  if (first_ball_there) {
    Shoot();
  } else {
    AOS_LOG(INFO, "Nah, not shooting\n");
  }

  AOS_LOG(INFO, "First shot at %f seconds\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  if (ShouldCancel()) return;

  SetShooterSpeed(0.0);
  AOS_LOG(INFO, "Folding superstructure back down\n");
  TuckArm(true, true);

  // Undo vision move.
  StartDrive(0.0, 0.0, kTwoBallFastDrive, kFinishTurn);
  if (!WaitForDriveDone()) return;

  constexpr double kBackDrive = 3.09 - 0.4;
  StartDrive(kBackDrive, 0.0, kTwoBallReturnDrive, kSlowTurn);
  if (!WaitForDriveNear(kBackDrive - 0.19, kDoNotTurnCare)) return;
  StartDrive(0, -kShootTurnAngle, kTwoBallReturnDrive, kSwerveTurn);
  if (!WaitForDriveNear(1.0, kDoNotTurnCare)) return;
  StartDrive(0, 0, kTwoBallReturnSlow, kSwerveTurn);

  if (!WaitForDriveNear(0.06, kDoNotTurnCare)) return;
  AOS_LOG(INFO, "At Low Bar %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));

  OpenShooter();
  constexpr double kSecondBallAfterBarDrive = 2.10;
  StartDrive(kSecondBallAfterBarDrive, 0.0, kTwoBallBallPickupAccel, kSlowTurn);
  if (!WaitForDriveNear(kSecondBallAfterBarDrive - 0.5, kDoNotTurnCare)) return;
  constexpr double kBallSmallWallTurn = -0.11;
  StartDrive(0, kBallSmallWallTurn, kTwoBallBallPickup, kFinishTurn);

  MoveSuperstructure(0.03, -0.010, 0.0, MakeProfileParameters(8.0, 60.0),
                     MakeProfileParameters(4.0, 10.0),
                     MakeProfileParameters(10.0, 25.0), false, 12.0);

  if (!WaitForDriveProfileDone()) return;

  MoveSuperstructure(0.10, -0.010, 0.0, MakeProfileParameters(8.0, 60.0),
                     MakeProfileParameters(4.0, 10.0),
                     MakeProfileParameters(10.0, 25.0), false, 12.0);

  AOS_LOG(INFO, "Done backing up %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));

  constexpr double kDriveBackDistance = 5.15 - 0.4;
  StartDrive(-kDriveBackDistance, 0.0, kTwoBallLowDrive, kFinishTurn);
  CloseIfBall();
  if (!WaitForDriveNear(kDriveBackDistance - 0.75, kDoNotTurnCare)) return;

  StartDrive(0.0, -kBallSmallWallTurn, kTwoBallLowDrive, kFinishTurn);
  AOS_LOG(INFO, "Straightening up at %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));

  CloseIfBall();
  if (!WaitForDriveNear(kDriveBackDistance - 2.3, kDoNotTurnCare)) return;

  ball_detector_fetcher_.Fetch();
  if (ball_detector_fetcher_.get()) {
    const bool ball_detected = ball_detector_fetcher_->voltage() > 2.5;
    if (!ball_detected) {
      if (!WaitForDriveDone()) return;
      AOS_LOG(INFO, "Aborting, no ball %f\n",
              ::aos::time::DurationInSeconds(monotonic_now() - start_time));
      return;
    }
  }
  CloseShooter();

  BackLongShotLowBarTwoBall();
  AOS_LOG(INFO, "Spinning up the shooter wheels\n");
  SetShooterSpeed(640.0);
  StartDrive(0.0, 0.0, kTwoBallFastDrive, kSwerveTurn);

  if (!WaitForDriveNear(1.80, kDoNotTurnCare)) return;
  StartDrive(0, kShootTurnAngle, kTwoBallFastDrive, kFinishTurn);
  BackLongShotTwoBall();

  if (!WaitForDriveDone()) return;
  AOS_LOG(INFO, "Second shot done driving at %f seconds\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  WaitForSuperstructure();
  AlignWithVisionGoal();
  if (ShouldCancel()) return;

  WaitForShooterSpeed();
  if (ShouldCancel()) return;

  // 2.2 with 0.4 of vision.
  // 1.8 without any vision.
  AOS_LOG(INFO, "Going to vision align at %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  WaitForAlignedWithVision(
      (start_time + chrono::milliseconds(13500) + kVisionExtra * 2) -
      monotonic_now());
  BackLongShotTwoBallFinish();
  WaitForSuperstructureProfile();
  if (ShouldCancel()) return;
  AOS_LOG(INFO, "Shoot at %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  Shoot();

  AOS_LOG(INFO, "Second shot at %f seconds\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
  if (ShouldCancel()) return;

  SetShooterSpeed(0.0);
  AOS_LOG(INFO, "Folding superstructure back down\n");
  TuckArm(true, false);
  AOS_LOG(INFO, "Shot %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));

  WaitForSuperstructureLow();

  AOS_LOG(INFO, "Done %f\n",
          ::aos::time::DurationInSeconds(monotonic_now() - start_time));
}

void AutonomousActor::StealAndMoveOverBy(double distance) {
  OpenShooter();
  MoveSuperstructure(0.10, -0.010, 0.0, MakeProfileParameters(8.0, 60.0),
                     MakeProfileParameters(4.0, 10.0),
                     MakeProfileParameters(10.0, 25.0), true, 12.0);
  if (ShouldCancel()) return;
  AOS_LOG(INFO, "Waiting for the intake to come down.\n");

  WaitForIntake();
  if (ShouldCancel()) return;
  StartDrive(-distance, M_PI / 2.0, kFastDrive, kStealTurn);
  WaitForBallOrDriveDone();
  if (ShouldCancel()) return;
  MoveSuperstructure(1.0, -0.010, 0.0, MakeProfileParameters(8.0, 60.0),
                     MakeProfileParameters(4.0, 10.0),
                     MakeProfileParameters(10.0, 25.0), true, 12.0);

  if (!WaitForDriveDone()) return;
  StartDrive(0.0, M_PI / 2.0, kFastDrive, kStealTurn);
  if (!WaitForDriveDone()) return;
}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams *params) {
  monotonic_clock::time_point start_time = monotonic_now();
  AOS_LOG(INFO, "Starting autonomous action with mode %" PRId32 "\n",
          params->mode());

  InitializeEncoders();
  ResetDrivetrain();

  switch (params->mode()) {
    case 0:
      LowBarDrive();
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      AOS_LOG(INFO, "Unfolding superstructure\n");
      FrontLongShot();

      // Spin up the shooter wheels.
      AOS_LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(640.0);

      break;
    case 1:
      TwoFromMiddleDrive();
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      AOS_LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      AOS_LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

      break;
    case 2:
      OneFromMiddleDrive(true);
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      AOS_LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      AOS_LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

      break;
    case 3:
      MiddleDrive();
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      AOS_LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      AOS_LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

      break;
    case 4:
      OneFromMiddleDrive(false);
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      AOS_LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      AOS_LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

      break;
    case 5:
    case 15:
      TwoBallAuto();
      return true;
      break;
    case 6:
      StealAndMoveOverBy(3.10 + 2 * 52 * 2.54 / 100.0);
      if (ShouldCancel()) return true;

      TwoFromMiddleDrive();
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      AOS_LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      AOS_LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

      break;
    case 7:
      StealAndMoveOverBy(2.95 + 52 * 2.54 / 100.0);
      if (ShouldCancel()) return true;

      OneFromMiddleDrive(true);
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      AOS_LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      AOS_LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

      break;
    case 8: {
      StealAndMoveOverBy(2.95);
      if (ShouldCancel()) return true;

      MiddleDrive();
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      AOS_LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      AOS_LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

    } break;
    case 9: {
      StealAndMoveOverBy(1.70);
      if (ShouldCancel()) return true;

      OneFromMiddleDrive(false);
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      AOS_LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      AOS_LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

    } break;
    default:
      AOS_LOG(ERROR, "Invalid auto mode %d\n", params->mode());
      return true;
  }

  DoFullShot();

  StartDrive(0.5, 0.0, kMoatDrive, kFastTurn);
  if (!WaitForDriveDone()) return true;

  AOS_LOG(INFO, "Done %f\n",
      ::aos::time::DurationInSeconds(monotonic_now() - start_time));

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);

  while (!ShouldCancel()) {
    phased_loop.SleepUntilNext();
  }
  AOS_LOG(DEBUG, "Done running\n");

  return true;
}

}  // namespace actors
}  // namespace y2016
