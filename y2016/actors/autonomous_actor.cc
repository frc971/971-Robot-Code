#include "y2016/actors/autonomous_actor.h"

#include <inttypes.h>

#include <chrono>
#include <cmath>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2016/control_loops/drivetrain/drivetrain_base.h"
#include "y2016/control_loops/shooter/shooter.q.h"
#include "y2016/control_loops/superstructure/superstructure.q.h"
#include "y2016/queues/ball_detector.q.h"
#include "y2016/vision/vision.q.h"

namespace y2016 {
namespace actors {
using ::frc971::control_loops::drivetrain_queue;
using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;
namespace this_thread = ::std::this_thread;

namespace {
const ProfileParameters kSlowDrive = {0.8, 2.5};
const ProfileParameters kLowBarDrive = {1.3, 2.5};
const ProfileParameters kMoatDrive = {1.2, 3.5};
const ProfileParameters kRealignDrive = {2.0, 2.5};
const ProfileParameters kRockWallDrive = {0.8, 2.5};
const ProfileParameters kFastDrive = {3.0, 2.5};

const ProfileParameters kSlowTurn = {0.8, 3.0};
const ProfileParameters kFastTurn = {3.0, 10.0};
const ProfileParameters kStealTurn = {4.0, 15.0};
const ProfileParameters kSwerveTurn = {2.0, 7.0};
const ProfileParameters kFinishTurn = {2.0, 5.0};

const ProfileParameters kTwoBallLowDrive = {1.7, 3.5};
const ProfileParameters kTwoBallFastDrive = {3.0, 1.5};
const ProfileParameters kTwoBallReturnDrive = {3.0, 1.9};
const ProfileParameters kTwoBallReturnSlow = {3.0, 2.5};
const ProfileParameters kTwoBallBallPickup = {2.0, 1.75};
const ProfileParameters kTwoBallBallPickupAccel = {2.0, 2.5};

double DoubleSeconds(monotonic_clock::duration duration) {
  return ::std::chrono::duration_cast<::std::chrono::duration<double>>(duration)
      .count();
}
}  // namespace

AutonomousActor::AutonomousActor(
    ::frc971::autonomous::AutonomousActionQueueGroup *s)
    : frc971::autonomous::BaseAutonomousActor(
          s, control_loops::drivetrain::GetDrivetrainConfig()) {}

constexpr double kDoNotTurnCare = 2.0;

void AutonomousActor::MoveSuperstructure(
    double intake, double shoulder, double wrist,
    const ProfileParameters intake_params,
    const ProfileParameters shoulder_params,
    const ProfileParameters wrist_params, bool traverse_up,
    double roller_power) {
  superstructure_goal_ = {intake, shoulder, wrist};

  auto new_superstructure_goal =
      ::y2016::control_loops::superstructure_queue.goal.MakeMessage();

  new_superstructure_goal->angle_intake = intake;
  new_superstructure_goal->angle_shoulder = shoulder;
  new_superstructure_goal->angle_wrist = wrist;

  new_superstructure_goal->max_angular_velocity_intake =
      intake_params.max_velocity;
  new_superstructure_goal->max_angular_velocity_shoulder =
      shoulder_params.max_velocity;
  new_superstructure_goal->max_angular_velocity_wrist =
      wrist_params.max_velocity;

  new_superstructure_goal->max_angular_acceleration_intake =
      intake_params.max_acceleration;
  new_superstructure_goal->max_angular_acceleration_shoulder =
      shoulder_params.max_acceleration;
  new_superstructure_goal->max_angular_acceleration_wrist =
      wrist_params.max_acceleration;

  new_superstructure_goal->voltage_top_rollers = roller_power;
  new_superstructure_goal->voltage_bottom_rollers = roller_power;

  new_superstructure_goal->traverse_unlatched = true;
  new_superstructure_goal->traverse_down = !traverse_up;
  new_superstructure_goal->voltage_climber = 0.0;
  new_superstructure_goal->unfold_climber = false;

  if (!new_superstructure_goal.Send()) {
    LOG(ERROR, "Sending superstructure goal failed.\n");
  }
}

void AutonomousActor::OpenShooter() {
  shooter_speed_ = 0.0;

  if (!control_loops::shooter::shooter_queue.goal.MakeWithBuilder()
           .angular_velocity(shooter_speed_)
           .clamp_open(true)
           .push_to_shooter(false)
           .force_lights_on(false)
           .Send()) {
    LOG(ERROR, "Sending shooter goal failed.\n");
  }
}

void AutonomousActor::CloseShooter() {
  shooter_speed_ = 0.0;

  if (!control_loops::shooter::shooter_queue.goal.MakeWithBuilder()
           .angular_velocity(shooter_speed_)
           .clamp_open(false)
           .push_to_shooter(false)
           .force_lights_on(false)
           .Send()) {
    LOG(ERROR, "Sending shooter goal failed.\n");
  }
}

void AutonomousActor::SetShooterSpeed(double speed) {
  shooter_speed_ = speed;

  // In auto, we want to have the lights on whenever possible since we have no
  // hope of a human aligning the robot.
  bool force_lights_on = shooter_speed_ > 1.0;

  if (!control_loops::shooter::shooter_queue.goal.MakeWithBuilder()
           .angular_velocity(shooter_speed_)
           .clamp_open(false)
           .push_to_shooter(false)
           .force_lights_on(force_lights_on)
           .Send()) {
    LOG(ERROR, "Sending shooter goal failed.\n");
  }
}

void AutonomousActor::Shoot() {
  uint32_t initial_shots = 0;

  control_loops::shooter::shooter_queue.status.FetchLatest();
  if (control_loops::shooter::shooter_queue.status.get()) {
    initial_shots = control_loops::shooter::shooter_queue.status->shots;
  }

  // In auto, we want to have the lights on whenever possible since we have no
  // hope of a human aligning the robot.
  bool force_lights_on = shooter_speed_ > 1.0;

  if (!control_loops::shooter::shooter_queue.goal.MakeWithBuilder()
           .angular_velocity(shooter_speed_)
           .clamp_open(false)
           .push_to_shooter(true)
           .force_lights_on(force_lights_on)
           .Send()) {
    LOG(ERROR, "Sending shooter goal failed.\n");
  }

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) return;

    // Wait for the shot count to change so we know when the shot is complete.
    control_loops::shooter::shooter_queue.status.FetchLatest();
    if (control_loops::shooter::shooter_queue.status.get()) {
      if (initial_shots < control_loops::shooter::shooter_queue.status->shots) {
        return;
      }
    }
    phased_loop.SleepUntilNext();
  }
}

void AutonomousActor::WaitForShooterSpeed() {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) return;

    control_loops::shooter::shooter_queue.status.FetchLatest();
    if (control_loops::shooter::shooter_queue.status.get()) {
      if (control_loops::shooter::shooter_queue.status->left.ready &&
          control_loops::shooter::shooter_queue.status->right.ready) {
        return;
      }
    }
    phased_loop.SleepUntilNext();
  }
}

void AutonomousActor::AlignWithVisionGoal() {
  actors::VisionAlignActionParams params;
  vision_action_ = ::std::move(actors::MakeVisionAlignAction(params));
  vision_action_->Start();
}

void AutonomousActor::WaitForAlignedWithVision(
    chrono::nanoseconds align_duration) {
  bool vision_valid = false;
  double last_angle = 0.0;
  int ready_to_fire = 0;

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  monotonic_clock::time_point end_time =
      monotonic_clock::now() + align_duration;
  while (end_time > monotonic_clock::now()) {
    if (ShouldCancel()) break;

    ::y2016::vision::vision_status.FetchLatest();
    if (::y2016::vision::vision_status.get()) {
      vision_valid = (::y2016::vision::vision_status->left_image_valid &&
                      ::y2016::vision::vision_status->right_image_valid);
      last_angle = ::y2016::vision::vision_status->horizontal_angle;
    }

    drivetrain_queue.status.FetchLatest();
    drivetrain_queue.goal.FetchLatest();

    if (drivetrain_queue.status.get() && drivetrain_queue.goal.get()) {
      const double left_goal = drivetrain_queue.goal->left_goal;
      const double right_goal = drivetrain_queue.goal->right_goal;
      const double left_current =
          drivetrain_queue.status->estimated_left_position;
      const double right_current =
          drivetrain_queue.status->estimated_right_position;
      const double left_velocity =
          drivetrain_queue.status->estimated_left_velocity;
      const double right_velocity =
          drivetrain_queue.status->estimated_right_velocity;

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
        LOG(INFO, "Vision align success!\n");
      }
    }
    phased_loop.SleepUntilNext();
  }

  vision_action_->Cancel();
  WaitUntilDoneOrCanceled(::std::move(vision_action_));
  LOG(INFO, "Done waiting for vision\n");
}

bool AutonomousActor::IntakeDone() {
  control_loops::superstructure_queue.status.FetchAnother();

  constexpr double kProfileError = 1e-5;
  constexpr double kEpsilon = 0.15;

  if (control_loops::superstructure_queue.status->state < 12 ||
      control_loops::superstructure_queue.status->state == 16) {
    LOG(ERROR, "Superstructure no longer running, aborting action\n");
    return true;
  }

  if (::std::abs(control_loops::superstructure_queue.status->intake.goal_angle -
                 superstructure_goal_.intake) < kProfileError &&
      ::std::abs(control_loops::superstructure_queue.status->intake
                     .goal_angular_velocity) < kProfileError) {
    LOG(DEBUG, "Profile done.\n");
    if (::std::abs(control_loops::superstructure_queue.status->intake.angle -
                   superstructure_goal_.intake) < kEpsilon &&
        ::std::abs(control_loops::superstructure_queue.status->intake
                       .angular_velocity) < kEpsilon) {
      LOG(INFO, "Near goal, done.\n");
      return true;
    }
  }
  return false;
}

bool AutonomousActor::SuperstructureProfileDone() {
  constexpr double kProfileError = 1e-5;
  return ::std::abs(
             control_loops::superstructure_queue.status->intake.goal_angle -
             superstructure_goal_.intake) < kProfileError &&
         ::std::abs(
             control_loops::superstructure_queue.status->shoulder.goal_angle -
             superstructure_goal_.shoulder) < kProfileError &&
         ::std::abs(
             control_loops::superstructure_queue.status->wrist.goal_angle -
             superstructure_goal_.wrist) < kProfileError &&
         ::std::abs(control_loops::superstructure_queue.status->intake
                        .goal_angular_velocity) < kProfileError &&
         ::std::abs(control_loops::superstructure_queue.status->shoulder
                        .goal_angular_velocity) < kProfileError &&
         ::std::abs(control_loops::superstructure_queue.status->wrist
                        .goal_angular_velocity) < kProfileError;
}

bool AutonomousActor::SuperstructureDone() {
  control_loops::superstructure_queue.status.FetchAnother();

  constexpr double kEpsilon = 0.03;

  if (control_loops::superstructure_queue.status->state < 12 ||
      control_loops::superstructure_queue.status->state == 16) {
    LOG(ERROR, "Superstructure no longer running, aborting action\n");
    return true;
  }

  if (SuperstructureProfileDone()) {
    LOG(DEBUG, "Profile done.\n");
    if (::std::abs(control_loops::superstructure_queue.status->intake.angle -
                   superstructure_goal_.intake) < (kEpsilon + 0.1) &&
        ::std::abs(control_loops::superstructure_queue.status->shoulder.angle -
                   superstructure_goal_.shoulder) < (kEpsilon + 0.05) &&
        ::std::abs(control_loops::superstructure_queue.status->wrist.angle -
                   superstructure_goal_.wrist) < (kEpsilon + 0.01) &&
        ::std::abs(control_loops::superstructure_queue.status->intake
                       .angular_velocity) < (kEpsilon + 0.1) &&
        ::std::abs(control_loops::superstructure_queue.status->shoulder
                       .angular_velocity) < (kEpsilon + 0.10) &&
        ::std::abs(control_loops::superstructure_queue.status->wrist
                       .angular_velocity) < (kEpsilon + 0.05)) {
      LOG(INFO, "Near goal, done.\n");
      return true;
    }
  }
  return false;
}

void AutonomousActor::WaitForIntake() {
  while (true) {
    if (ShouldCancel()) return;
    if (IntakeDone()) return;
  }
}

void AutonomousActor::WaitForSuperstructure() {
  while (true) {
    if (ShouldCancel()) return;
    if (SuperstructureDone()) return;
  }
}

void AutonomousActor::WaitForSuperstructureProfile() {
  while (true) {
    if (ShouldCancel()) return;
    control_loops::superstructure_queue.status.FetchAnother();

    if (control_loops::superstructure_queue.status->state < 12 ||
        control_loops::superstructure_queue.status->state == 16) {
      LOG(ERROR, "Superstructure no longer running, aborting action\n");
      return;
    }

    if (SuperstructureProfileDone()) return;
  }
}

void AutonomousActor::WaitForSuperstructureLow() {
  while (true) {
    if (ShouldCancel()) return;
    control_loops::superstructure_queue.status.FetchAnother();

    if (control_loops::superstructure_queue.status->state < 12 ||
        control_loops::superstructure_queue.status->state == 16) {
      LOG(ERROR, "Superstructure no longer running, aborting action\n");
      return;
    }
    if (SuperstructureProfileDone()) return;
    if (control_loops::superstructure_queue.status->shoulder.angle < 0.1) {
      return;
    }
  }
}
void AutonomousActor::BackLongShotLowBarTwoBall() {
  LOG(INFO, "Expanding for back long shot\n");
  MoveSuperstructure(0.00, M_PI / 2.0 - 0.2, -0.55, {7.0, 40.0}, {4.0, 6.0},
                     {10.0, 25.0}, false, 0.0);
}

void AutonomousActor::BackLongShotTwoBall() {
  LOG(INFO, "Expanding for back long shot\n");
  MoveSuperstructure(0.00, M_PI / 2.0 - 0.2, -0.55, {7.0, 40.0}, {4.0, 6.0},
                     {10.0, 25.0}, false, 0.0);
}

void AutonomousActor::BackLongShotTwoBallFinish() {
  LOG(INFO, "Expanding for back long shot\n");
  MoveSuperstructure(0.00, M_PI / 2.0 - 0.2, -0.625 + 0.03, {7.0, 40.0},
                     {4.0, 6.0}, {10.0, 25.0}, false, 0.0);
}

void AutonomousActor::BackLongShot() {
  LOG(INFO, "Expanding for back long shot\n");
  MoveSuperstructure(0.80, M_PI / 2.0 - 0.2, -0.62, {7.0, 40.0}, {4.0, 6.0},
                     {10.0, 25.0}, false, 0.0);
}

void AutonomousActor::BackMiddleShot() {
  LOG(INFO, "Expanding for back middle shot\n");
  MoveSuperstructure(-0.05, M_PI / 2.0 - 0.2, -0.665, {7.0, 40.0}, {4.0, 10.0},
                     {10.0, 25.0}, false, 0.0);
}

void AutonomousActor::FrontLongShot() {
  LOG(INFO, "Expanding for front long shot\n");
  MoveSuperstructure(0.80, M_PI / 2.0 + 0.1, M_PI + 0.41 + 0.02, {7.0, 40.0},
                     {4.0, 6.0}, {10.0, 25.0}, false, 0.0);
}

void AutonomousActor::FrontMiddleShot() {
  LOG(INFO, "Expanding for front middle shot\n");
  MoveSuperstructure(-0.05, M_PI / 2.0 + 0.1, M_PI + 0.44, {7.0, 40.0},
                     {4.0, 10.0}, {10.0, 25.0}, true, 0.0);
}

void AutonomousActor::TuckArm(bool low_bar, bool traverse_down) {
  MoveSuperstructure(low_bar ? -0.05 : 2.0, -0.010, 0.0, {7.0, 40.0},
                     {4.0, 10.0}, {10.0, 25.0}, !traverse_down, 0.0);
}

void AutonomousActor::DoFullShot() {
  if (ShouldCancel()) return;
  // Make sure that the base is aligned with the base.
  LOG(INFO, "Waiting for the superstructure\n");
  WaitForSuperstructure();

  this_thread::sleep_for(chrono::milliseconds(500));

  if (ShouldCancel()) return;
  LOG(INFO, "Triggering the vision actor\n");
  AlignWithVisionGoal();

  // Wait for the drive base to be aligned with the target and make sure that
  // the shooter is up to speed.
  LOG(INFO, "Waiting for vision to be aligned\n");
  WaitForAlignedWithVision(chrono::milliseconds(2000));
  if (ShouldCancel()) return;
  LOG(INFO, "Waiting for shooter to be up to speed\n");
  WaitForShooterSpeed();
  if (ShouldCancel()) return;

  this_thread::sleep_for(chrono::milliseconds(300));
  LOG(INFO, "Shoot!\n");
  Shoot();

  // Turn off the shooter and fold up the superstructure.
  if (ShouldCancel()) return;
  LOG(INFO, "Stopping shooter\n");
  SetShooterSpeed(0.0);
  LOG(INFO, "Folding superstructure back down\n");
  TuckArm(false, false);

  // Wait for everything to be folded up.
  LOG(INFO, "Waiting for superstructure to be folded back down\n");
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
  drivetrain_queue.status.FetchLatest();
  if (drivetrain_queue.status.get()) {
    const double left_error =
        (initial_drivetrain_.left -
         drivetrain_queue.status->estimated_left_position);
    const double right_error =
        (initial_drivetrain_.right -
         drivetrain_queue.status->estimated_right_position);
    const double distance_to_go = (left_error + right_error) / 2.0;
    const double distance_compensation =
        goal_distance - tip_distance - distance_to_go;
    LOG(INFO, "Going %f further at the bump\n", distance_compensation);
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
  ::y2016::sensors::ball_detector.FetchLatest();
  if (::y2016::sensors::ball_detector.get()) {
    const bool ball_detected = ::y2016::sensors::ball_detector->voltage > 2.5;
    if (ball_detected) {
      CloseShooter();
    }
  }
}

void AutonomousActor::WaitForBallOrDriveDone() {
  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    if (ShouldCancel()) {
      return;
    }
    phased_loop.SleepUntilNext();
    drivetrain_queue.status.FetchLatest();
    if (IsDriveDone()) {
      return;
    }

    ::y2016::sensors::ball_detector.FetchLatest();
    if (::y2016::sensors::ball_detector.get()) {
      const bool ball_detected = ::y2016::sensors::ball_detector->voltage > 2.5;
      if (ball_detected) {
        return;
      }
    }
  }
}

void AutonomousActor::WaitForBall() {
  while (true) {
    ::y2016::sensors::ball_detector.FetchAnother();
    if (::y2016::sensors::ball_detector.get()) {
      const bool ball_detected = ::y2016::sensors::ball_detector->voltage > 2.5;
      if (ball_detected) {
        return;
      }
      if (ShouldCancel()) return;
    }
  }
}

void AutonomousActor::TwoBallAuto() {
  monotonic_clock::time_point start_time = monotonic_clock::now();
  OpenShooter();
  MoveSuperstructure(0.10, -0.010, 0.0, {8.0, 60.0}, {4.0, 10.0}, {10.0, 25.0},
                     false, 12.0);
  if (ShouldCancel()) return;
  LOG(INFO, "Waiting for the intake to come down.\n");

  WaitForIntake();
  LOG(INFO, "Intake done at %f seconds, starting to drive\n",
      DoubleSeconds(monotonic_clock::now() - start_time));
  if (ShouldCancel()) return;
  const double kDriveDistance = 5.05;
  StartDrive(-kDriveDistance, 0.0, kTwoBallLowDrive, kSlowTurn);

  StartDrive(0.0, 0.4, kTwoBallLowDrive, kSwerveTurn);
  if (!WaitForDriveNear(kDriveDistance - 0.5, kDoNotTurnCare)) return;

  // Check if the ball is there.
  bool first_ball_there = true;
  ::y2016::sensors::ball_detector.FetchLatest();
  if (::y2016::sensors::ball_detector.get()) {
    const bool ball_detected = ::y2016::sensors::ball_detector->voltage > 2.5;
    first_ball_there = ball_detected;
    LOG(INFO, "Saw the ball: %d at %f\n", first_ball_there,
        DoubleSeconds(monotonic_clock::now() - start_time));
  }
  MoveSuperstructure(0.10, -0.010, 0.0, {8.0, 40.0}, {4.0, 10.0}, {10.0, 25.0},
                     false, 0.0);
  LOG(INFO, "Shutting off rollers at %f seconds, starting to straighten out\n",
      DoubleSeconds(monotonic_clock::now() - start_time));
  StartDrive(0.0, -0.4, kTwoBallLowDrive, kSwerveTurn);
  MoveSuperstructure(-0.05, -0.010, 0.0, {8.0, 40.0}, {4.0, 10.0}, {10.0, 25.0},
                     false, 0.0);
  CloseShooter();
  if (!WaitForDriveNear(kDriveDistance - 2.4, kDoNotTurnCare)) return;

  // We are now under the low bar.  Start lifting.
  BackLongShotLowBarTwoBall();
  LOG(INFO, "Spinning up the shooter wheels\n");
  SetShooterSpeed(640.0);
  StartDrive(0.0, 0.0, kTwoBallFastDrive, kSwerveTurn);

  if (!WaitForDriveNear(1.50, kDoNotTurnCare)) return;
  constexpr double kShootTurnAngle = -M_PI / 4.0 - 0.05;
  StartDrive(0, kShootTurnAngle, kTwoBallFastDrive, kFinishTurn);
  BackLongShotTwoBall();

  if (!WaitForDriveDone()) return;
  LOG(INFO, "First shot done driving at %f seconds\n",
      DoubleSeconds(monotonic_clock::now() - start_time));

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
  LOG(INFO, "Shoot!\n");
  if (first_ball_there) {
    Shoot();
  } else {
    LOG(INFO, "Nah, not shooting\n");
  }

  LOG(INFO, "First shot at %f seconds\n",
      DoubleSeconds(monotonic_clock::now() - start_time));
  if (ShouldCancel()) return;

  SetShooterSpeed(0.0);
  LOG(INFO, "Folding superstructure back down\n");
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
  LOG(INFO, "At Low Bar %f\n",
      DoubleSeconds(monotonic_clock::now() - start_time));

  OpenShooter();
  constexpr double kSecondBallAfterBarDrive = 2.10;
  StartDrive(kSecondBallAfterBarDrive, 0.0, kTwoBallBallPickupAccel, kSlowTurn);
  if (!WaitForDriveNear(kSecondBallAfterBarDrive - 0.5, kDoNotTurnCare)) return;
  constexpr double kBallSmallWallTurn = -0.11;
  StartDrive(0, kBallSmallWallTurn, kTwoBallBallPickup, kFinishTurn);

  MoveSuperstructure(0.03, -0.010, 0.0, {8.0, 60.0}, {4.0, 10.0}, {10.0, 25.0},
                     false, 12.0);

  if (!WaitForDriveProfileDone()) return;

  MoveSuperstructure(0.10, -0.010, 0.0, {8.0, 60.0}, {4.0, 10.0}, {10.0, 25.0},
                     false, 12.0);

  LOG(INFO, "Done backing up %f\n",
      DoubleSeconds(monotonic_clock::now() - start_time));

  constexpr double kDriveBackDistance = 5.15 - 0.4;
  StartDrive(-kDriveBackDistance, 0.0, kTwoBallLowDrive, kFinishTurn);
  CloseIfBall();
  if (!WaitForDriveNear(kDriveBackDistance - 0.75, kDoNotTurnCare)) return;

  StartDrive(0.0, -kBallSmallWallTurn, kTwoBallLowDrive, kFinishTurn);
  LOG(INFO, "Straightening up at %f\n",
      DoubleSeconds(monotonic_clock::now() - start_time));

  CloseIfBall();
  if (!WaitForDriveNear(kDriveBackDistance - 2.3, kDoNotTurnCare)) return;

  ::y2016::sensors::ball_detector.FetchLatest();
  if (::y2016::sensors::ball_detector.get()) {
    const bool ball_detected = ::y2016::sensors::ball_detector->voltage > 2.5;
    if (!ball_detected) {
      if (!WaitForDriveDone()) return;
      LOG(INFO, "Aborting, no ball %f\n",
          DoubleSeconds(monotonic_clock::now() - start_time));
      return;
    }
  }
  CloseShooter();

  BackLongShotLowBarTwoBall();
  LOG(INFO, "Spinning up the shooter wheels\n");
  SetShooterSpeed(640.0);
  StartDrive(0.0, 0.0, kTwoBallFastDrive, kSwerveTurn);

  if (!WaitForDriveNear(1.80, kDoNotTurnCare)) return;
  StartDrive(0, kShootTurnAngle, kTwoBallFastDrive, kFinishTurn);
  BackLongShotTwoBall();

  if (!WaitForDriveDone()) return;
  LOG(INFO, "Second shot done driving at %f seconds\n",
      DoubleSeconds(monotonic_clock::now() - start_time));
  WaitForSuperstructure();
  AlignWithVisionGoal();
  if (ShouldCancel()) return;

  WaitForShooterSpeed();
  if (ShouldCancel()) return;

  // 2.2 with 0.4 of vision.
  // 1.8 without any vision.
  LOG(INFO, "Going to vision align at %f\n",
      DoubleSeconds(monotonic_clock::now() - start_time));
  WaitForAlignedWithVision(
      (start_time + chrono::milliseconds(13500) + kVisionExtra * 2) -
      monotonic_clock::now());
  BackLongShotTwoBallFinish();
  WaitForSuperstructureProfile();
  if (ShouldCancel()) return;
  LOG(INFO, "Shoot at %f\n",
      DoubleSeconds(monotonic_clock::now() - start_time));
  Shoot();

  LOG(INFO, "Second shot at %f seconds\n",
      DoubleSeconds(monotonic_clock::now() - start_time));
  if (ShouldCancel()) return;

  SetShooterSpeed(0.0);
  LOG(INFO, "Folding superstructure back down\n");
  TuckArm(true, false);
  LOG(INFO, "Shot %f\n", DoubleSeconds(monotonic_clock::now() - start_time));

  WaitForSuperstructureLow();

  LOG(INFO, "Done %f\n", DoubleSeconds(monotonic_clock::now() - start_time));
}

void AutonomousActor::StealAndMoveOverBy(double distance) {
  OpenShooter();
  MoveSuperstructure(0.10, -0.010, 0.0, {8.0, 60.0}, {4.0, 10.0}, {10.0, 25.0},
                     true, 12.0);
  if (ShouldCancel()) return;
  LOG(INFO, "Waiting for the intake to come down.\n");

  WaitForIntake();
  if (ShouldCancel()) return;
  StartDrive(-distance, M_PI / 2.0, kFastDrive, kStealTurn);
  WaitForBallOrDriveDone();
  if (ShouldCancel()) return;
  MoveSuperstructure(1.0, -0.010, 0.0, {8.0, 60.0}, {4.0, 10.0}, {10.0, 25.0},
                     true, 12.0);

  if (!WaitForDriveDone()) return;
  StartDrive(0.0, M_PI / 2.0, kFastDrive, kStealTurn);
  if (!WaitForDriveDone()) return;
}

bool AutonomousActor::RunAction(
    const ::frc971::autonomous::AutonomousActionParams &params) {
  monotonic_clock::time_point start_time = monotonic_clock::now();
  LOG(INFO, "Starting autonomous action with mode %" PRId32 "\n", params.mode);

  InitializeEncoders();
  ResetDrivetrain();

  switch (params.mode) {
    case 0:
      LowBarDrive();
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      LOG(INFO, "Unfolding superstructure\n");
      FrontLongShot();

      // Spin up the shooter wheels.
      LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(640.0);

      break;
    case 1:
      TwoFromMiddleDrive();
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

      break;
    case 2:
      OneFromMiddleDrive(true);
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

      break;
    case 3:
      MiddleDrive();
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

      break;
    case 4:
      OneFromMiddleDrive(false);
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      LOG(INFO, "Spinning up the shooter wheels\n");
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
      LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

      break;
    case 7:
      StealAndMoveOverBy(2.95 + 52 * 2.54 / 100.0);
      if (ShouldCancel()) return true;

      OneFromMiddleDrive(true);
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

      break;
    case 8: {
      StealAndMoveOverBy(2.95);
      if (ShouldCancel()) return true;

      MiddleDrive();
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

    } break;
    case 9: {
      StealAndMoveOverBy(1.70);
      if (ShouldCancel()) return true;

      OneFromMiddleDrive(false);
      if (!WaitForDriveDone()) return true;
      // Get the superstructure to unfold and get ready for shooting.
      LOG(INFO, "Unfolding superstructure\n");
      FrontMiddleShot();

      // Spin up the shooter wheels.
      LOG(INFO, "Spinning up the shooter wheels\n");
      SetShooterSpeed(600.0);

    } break;
    default:
      LOG(ERROR, "Invalid auto mode %d\n", params.mode);
      return true;
  }

  DoFullShot();

  StartDrive(0.5, 0.0, kMoatDrive, kFastTurn);
  if (!WaitForDriveDone()) return true;

  LOG(INFO, "Done %f\n", DoubleSeconds(monotonic_clock::now() - start_time));

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);

  while (!ShouldCancel()) {
    phased_loop.SleepUntilNext();
  }
  LOG(DEBUG, "Done running\n");

  return true;
}

}  // namespace actors
}  // namespace y2016
