#include "y2016/actors/autonomous_actor.h"

#include <inttypes.h>

#include <cmath>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2016/control_loops/drivetrain/drivetrain_base.h"
#include "y2016/control_loops/shooter/shooter.q.h"
#include "y2016/control_loops/superstructure/superstructure.q.h"
#include "y2016/actors/autonomous_action.q.h"
#include "y2016/vision/vision.q.h"

namespace y2016 {
namespace actors {
using ::frc971::control_loops::drivetrain_queue;

namespace {
const ProfileParameters kSlowDrive = {0.8, 2.5};
const ProfileParameters kLowBarDrive = {1.3, 2.5};
const ProfileParameters kMoatDrive = {1.2, 3.5};
const ProfileParameters kRealignDrive = {2.0, 2.5};
const ProfileParameters kRockWallDrive = {0.8, 2.5};
const ProfileParameters kFastDrive = {3.0, 2.5};

const ProfileParameters kSlowTurn = {0.8, 3.0};
const ProfileParameters kFastTurn = {3.0, 10.0};

const double kDistanceShort = 0.25;
}  // namespace

AutonomousActor::AutonomousActor(actors::AutonomousActionQueueGroup *s)
    : aos::common::actions::ActorBase<actors::AutonomousActionQueueGroup>(s),
      dt_config_(control_loops::drivetrain::GetDrivetrainConfig()),
      initial_drivetrain_({0.0, 0.0}) {}

void AutonomousActor::ResetDrivetrain() {
  LOG(INFO, "resetting the drivetrain\n");
  drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .highgear(true)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(initial_drivetrain_.left)
      .left_velocity_goal(0)
      .right_goal(initial_drivetrain_.right)
      .right_velocity_goal(0)
      .Send();
}

void AutonomousActor::StartDrive(double distance, double angle,
                                 ProfileParameters linear,
                                 ProfileParameters angular) {
  {
    {
      const double dangle = angle * dt_config_.robot_radius;
      initial_drivetrain_.left += distance - dangle;
      initial_drivetrain_.right += distance + dangle;
    }

    auto drivetrain_message = drivetrain_queue.goal.MakeMessage();
    drivetrain_message->control_loop_driving = true;
    drivetrain_message->highgear = true;
    drivetrain_message->steering = 0.0;
    drivetrain_message->throttle = 0.0;
    drivetrain_message->left_goal = initial_drivetrain_.left;
    drivetrain_message->left_velocity_goal = 0;
    drivetrain_message->right_goal = initial_drivetrain_.right;
    drivetrain_message->right_velocity_goal = 0;
    drivetrain_message->linear = linear;
    drivetrain_message->angular = angular;

    LOG_STRUCT(DEBUG, "drivetrain_goal", *drivetrain_message);

    drivetrain_message.Send();
  }
}

void AutonomousActor::InitializeEncoders() {
  drivetrain_queue.status.FetchAnother();
  initial_drivetrain_.left = drivetrain_queue.status->estimated_left_position;
  initial_drivetrain_.right = drivetrain_queue.status->estimated_right_position;
}

void AutonomousActor::WaitUntilDoneOrCanceled(
    ::std::unique_ptr<aos::common::actions::Action> action) {
  if (!action) {
    LOG(ERROR, "No action, not waiting\n");
    return;
  }

  ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(5),
                                      ::aos::time::Time::InMS(5) / 2);
  while (true) {
    // Poll the running bit and see if we should cancel.
    phased_loop.SleepUntilNext();
    if (!action->Running() || ShouldCancel()) {
      return;
    }
  }
}

bool AutonomousActor::WaitForDriveNear(double distance, double angle) {
  ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(5),
                                      ::aos::time::Time::InMS(5) / 2);
  constexpr double kPositionTolerance = 0.02;
  constexpr double kProfileTolerance = 0.001;

  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_queue.status.FetchLatest();
    if (drivetrain_queue.status.get()) {
      const double left_profile_error =
          (initial_drivetrain_.left -
           drivetrain_queue.status->profiled_left_position_goal);
      const double right_profile_error =
          (initial_drivetrain_.right -
           drivetrain_queue.status->profiled_right_position_goal);

      const double left_error =
          (initial_drivetrain_.left -
           drivetrain_queue.status->estimated_left_position);
      const double right_error =
          (initial_drivetrain_.right -
           drivetrain_queue.status->estimated_right_position);

      const double profile_distance_to_go =
          (left_profile_error + right_profile_error) / 2.0;
      const double profile_angle_to_go =
          (right_profile_error - left_profile_error) /
          (dt_config_.robot_radius * 2.0);

      const double distance_to_go = (left_error + right_error) / 2.0;
      const double angle_to_go =
          (right_error - left_error) / (dt_config_.robot_radius * 2.0);

      if (::std::abs(profile_distance_to_go) < distance + kProfileTolerance &&
          ::std::abs(profile_angle_to_go) < angle + kProfileTolerance &&
          ::std::abs(distance_to_go) < distance + kPositionTolerance &&
          ::std::abs(angle_to_go) < angle + kPositionTolerance) {
        LOG(INFO, "Closer than %f distance, %f angle\n", distance, angle);
        return true;
      }
    }
  }
}

bool AutonomousActor::WaitForDriveDone() {
  ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(5),
                                      ::aos::time::Time::InMS(5) / 2);
  constexpr double kPositionTolerance = 0.02;
  constexpr double kVelocityTolerance = 0.10;
  constexpr double kProfileTolerance = 0.001;

  while (true) {
    if (ShouldCancel()) {
      return false;
    }
    phased_loop.SleepUntilNext();
    drivetrain_queue.status.FetchLatest();
    if (drivetrain_queue.status.get()) {
      if (::std::abs(drivetrain_queue.status->profiled_left_position_goal -
                     initial_drivetrain_.left) < kProfileTolerance &&
          ::std::abs(drivetrain_queue.status->profiled_right_position_goal -
                     initial_drivetrain_.right) < kProfileTolerance &&
          ::std::abs(drivetrain_queue.status->estimated_left_position -
                     initial_drivetrain_.left) < kPositionTolerance &&
          ::std::abs(drivetrain_queue.status->estimated_right_position -
                     initial_drivetrain_.right) < kPositionTolerance &&
          ::std::abs(drivetrain_queue.status->estimated_left_velocity) <
              kVelocityTolerance &&
          ::std::abs(drivetrain_queue.status->estimated_right_velocity) <
              kVelocityTolerance) {
        LOG(INFO, "Finished drive\n");
        return true;
      }
    }
  }
}

void AutonomousActor::MoveSuperstructure(
    double intake, double shoulder, double wrist,
    const ProfileParameters intake_params,
    const ProfileParameters shoulder_params,
    const ProfileParameters wrist_params, bool traverse_up) {
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

  new_superstructure_goal->voltage_top_rollers = 0.0;
  new_superstructure_goal->voltage_bottom_rollers = 0.0;

  new_superstructure_goal->traverse_unlatched = true;
  new_superstructure_goal->traverse_down = !traverse_up;

  if (!new_superstructure_goal.Send()) {
    LOG(ERROR, "Sending superstructure goal failed.\n");
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

  ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(5),
                                      ::aos::time::Time::InMS(5) / 2);
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
  ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(5),
                                      ::aos::time::Time::InMS(5) / 2);
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

void AutonomousActor::WaitForAlignedWithVision() {
  bool vision_valid = false;
  double last_angle = 0.0;
  int ready_to_fire = 0;

  ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(5),
                                      ::aos::time::Time::InMS(5) / 2);
  ::aos::time::Time end_time =
      ::aos::time::Time::Now() + aos::time::Time::InSeconds(3);
  while (end_time > ::aos::time::Time::Now()) {
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
      if (ready_to_fire > 9) {
        break;
      }
    }
    phased_loop.SleepUntilNext();
  }

  vision_action_->Cancel();
  WaitUntilDoneOrCanceled(::std::move(vision_action_));
}

void AutonomousActor::WaitForSuperstructure() {
  while (true) {
    if (ShouldCancel()) return;
    control_loops::superstructure_queue.status.FetchAnother();

    constexpr double kProfileError = 1e-5;
    constexpr double kEpsilon = 0.03;

    if (control_loops::superstructure_queue.status->state < 12 ||
        control_loops::superstructure_queue.status->state == 16) {
      LOG(ERROR, "Superstructure no longer running, aborting action\n");
      return;
    }

    if (::std::abs(
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
                       .goal_angular_velocity) < kProfileError) {
      LOG(INFO, "Profile done.\n");
      if (::std::abs(control_loops::superstructure_queue.status->intake.angle -
                     superstructure_goal_.intake) < kEpsilon &&
          ::std::abs(
              control_loops::superstructure_queue.status->shoulder.angle -
              superstructure_goal_.shoulder) < kEpsilon &&
          ::std::abs(control_loops::superstructure_queue.status->wrist.angle -
                     superstructure_goal_.wrist) < kEpsilon &&
          ::std::abs(control_loops::superstructure_queue.status->intake
                         .angular_velocity) < kEpsilon &&
          ::std::abs(control_loops::superstructure_queue.status->shoulder
                         .angular_velocity) < kEpsilon &&
          ::std::abs(control_loops::superstructure_queue.status->wrist
                         .angular_velocity) < kEpsilon) {
        LOG(INFO, "Near goal, done.\n");
        return;
      }
    }
  }
}
void AutonomousActor::BackLongShot() {
  LOG(INFO, "Expanding for back long shot\n");
  MoveSuperstructure(-0.05, M_PI / 2.0 - 0.2, -0.62, {7.0, 40.0}, {4.0, 10.0},
                     {10.0, 25.0}, false);
}

void AutonomousActor::BackMiddleShot() {
  LOG(INFO, "Expanding for back middle shot\n");
  MoveSuperstructure(-0.05, M_PI / 2.0 - 0.2, -0.665, {7.0, 40.0}, {4.0, 10.0},
                     {10.0, 25.0}, false);
}

void AutonomousActor::TuckArm(bool low_bar, bool traverse_down) {
  MoveSuperstructure(low_bar ? -0.05 : 2.0, -0.010, 0.0, {7.0, 40.0},
                     {4.0, 10.0}, {10.0, 25.0}, !traverse_down);
}

void AutonomousActor::DoFullShot(bool center) {
  // Get the superstructure to unfold and get ready for shooting.
  LOG(INFO, "Unfolding superstructure\n");
  if (center) {
    BackMiddleShot();
  } else {
    BackLongShot();
  }

  // Spin up the shooter wheels.
  LOG(INFO, "Spinning up the shooter wheels\n");
  SetShooterSpeed(640.0);

  if (ShouldCancel()) return;
  // Make sure that the base is aligned with the base.
  LOG(INFO, "Waiting for the superstructure\n");
  WaitForSuperstructure();
  if (ShouldCancel()) return;
  LOG(INFO, "Triggering the vision actor\n");
  AlignWithVisionGoal();

  // Wait for the drive base to be aligned with the target and make sure that
  // the shooter is up to speed.
  LOG(INFO, "Waiting for vision to be aligned\n");
  WaitForAlignedWithVision();
  if (ShouldCancel()) return;
  LOG(INFO, "Waiting for shooter to be up to speed\n");
  WaitForShooterSpeed();
  if (ShouldCancel()) return;

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
  WaitForSuperstructure();
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

  StartDrive(0, -M_PI / 4.0 - 0.1, kLowBarDrive, kSlowTurn);
}

void AutonomousActor::MiddleDrive() {
  TuckArm(false, false);
  StartDrive(4.7, 0.0, kMoatDrive, kSlowTurn);
  if (!WaitForDriveDone()) return;
  StartDrive(0.0, M_PI, kMoatDrive, kFastTurn);
}

void AutonomousActor::OneFromMiddleDrive(bool left) {
  const double kTurnAngle = left ? M_PI / 2.0 - 0.40 : (-M_PI / 2.0 + 0.40);
  const double kFlipTurnAngle =
      left ? (M_PI - kTurnAngle) : (-M_PI - kTurnAngle);
  TuckArm(false, false);
  StartDrive(5.0 - kDistanceShort, 0.0, kMoatDrive, kFastTurn);
  if (!WaitForDriveDone()) return;
  StartDrive(0.0, kTurnAngle, kRealignDrive, kFastTurn);
  if (!WaitForDriveDone()) return;
  StartDrive(-1.3, 0.0, kRealignDrive, kFastTurn);
  if (!WaitForDriveDone()) return;
  StartDrive(0.0, kFlipTurnAngle, kRealignDrive, kFastTurn);

  if (!WaitForDriveDone()) return;
  StartDrive(0.3, 0.0, kRealignDrive, kFastTurn);
}

void AutonomousActor::TwoFromMiddleDrive() {
  const double kTurnAngle = M_PI / 2.0 - 0.13;
  TuckArm(false, false);
  StartDrive(5.0 - kDistanceShort, 0.0, kMoatDrive, kFastTurn);
  if (!WaitForDriveDone()) return;
  StartDrive(0.0, kTurnAngle, kMoatDrive, kFastTurn);
  if (!WaitForDriveDone()) return;
  StartDrive(-2.6, 0.0, kRealignDrive, kFastTurn);
  if (!WaitForDriveDone()) return;
  StartDrive(0.0, M_PI - kTurnAngle, kMoatDrive, kFastTurn);

  if (!WaitForDriveDone()) return;
  StartDrive(0.3, 0.0, kRealignDrive, kFastTurn);
}

bool AutonomousActor::RunAction(const actors::AutonomousActionParams &params) {
  LOG(INFO, "Starting autonomous action with mode %" PRId32 "\n", params.mode);

  InitializeEncoders();
  ResetDrivetrain();

  int index = 0;
  switch (index) {
    case 0:
      LowBarDrive();
      break;
    case 1:
      TwoFromMiddleDrive();
      break;
    case 2:
      OneFromMiddleDrive(true);
      break;
    case 3:
      MiddleDrive();
      break;
    case 4:
      OneFromMiddleDrive(false);
      break;
    default:
      LOG(ERROR, "Invalid auto index %d\n", index);
      return true;
  }

  if (!WaitForDriveDone()) return true;

  DoFullShot(index != 0);

  ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(5),
                                      ::aos::time::Time::InMS(5) / 2);

  while (!ShouldCancel()) {
    phased_loop.SleepUntilNext();
  }
  LOG(DEBUG, "Done running\n");

  return true;
}

::std::unique_ptr<AutonomousAction> MakeAutonomousAction(
    const ::y2016::actors::AutonomousActionParams &params) {
  return ::std::unique_ptr<AutonomousAction>(
      new AutonomousAction(&::y2016::actors::autonomous_action, params));
}

}  // namespace actors
}  // namespace y2016
