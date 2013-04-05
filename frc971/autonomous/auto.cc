#include "stdio.h"

#include "aos/aos_core.h"
#include "aos/common/control_loop/Timing.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "frc971/autonomous/auto.q.h"
#include "aos/common/messages/RobotState.q.h"
#include "frc971/constants.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/wrist/wrist_motor.q.h"
#include "frc971/control_loops/index/index_motor.q.h"
#include "frc971/control_loops/shooter/shooter_motor.q.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor.q.h"

using ::aos::time::Time;

namespace frc971 {
namespace autonomous {

static double left_initial_position, right_initial_position;

bool ShouldExitAuto() {
  ::frc971::autonomous::autonomous.FetchLatest();
  bool ans = !::frc971::autonomous::autonomous->run_auto;
  if (ans) {
    LOG(INFO, "Time to exit auto mode\n");
  }
  return ans;
}

void SetShooterVelocity(double velocity) {
  LOG(INFO, "Setting shooter velocity to %f\n", velocity);
  control_loops::shooter.goal.MakeWithBuilder()
    .velocity(velocity).Send();
}

void SetWristGoal(double goal) {
  LOG(INFO, "Setting wrist to %f\n", goal);
  control_loops::wrist.goal.MakeWithBuilder()
    .goal(goal).Send();
}

void SetAngle_AdjustGoal(double goal) {
  LOG(INFO, "Setting angle adjust to %f\n", goal);
  control_loops::angle_adjust.goal.MakeWithBuilder()
    .goal(goal).Send();
}

void StartIndex() {
  LOG(INFO, "Starting index\n");
  control_loops::index_loop.goal.MakeWithBuilder()
    .goal_state(2).Send();
}

void PreloadIndex() {
  LOG(INFO, "Preloading index\n");
  control_loops::index_loop.goal.MakeWithBuilder()
    .goal_state(3).Send();
}

void ShootIndex() {
  LOG(INFO, "Shooting index\n");
  control_loops::index_loop.goal.MakeWithBuilder()
    .goal_state(4).Send();
}

void ResetIndex() {
  LOG(INFO, "Resetting index\n");
  control_loops::index_loop.goal.MakeWithBuilder()
    .goal_state(5).Send();
}

void WaitForIndexReset() {
  LOG(INFO, "Waiting for the indexer to reset\n");
  control_loops::index_loop.status.FetchLatest();

  // Fetch a couple index status packets to make sure that the indexer has run.
  for (int i = 0; i < 5; ++i) {
    LOG(DEBUG, "Fetching another index status packet\n");
    control_loops::index_loop.status.FetchNextBlocking();
    if (ShouldExitAuto()) return;
  }
  LOG(INFO, "Indexer is now reset.\n");
}

void WaitForWrist() {
  LOG(INFO, "Waiting for the wrist\n");
  control_loops::wrist.status.FetchLatest();

  while (!control_loops::wrist.status.get()) {
    LOG(WARNING, "No previous wrist packet, trying to fetch again\n");
    control_loops::wrist.status.FetchNextBlocking();
  }

  while (!control_loops::wrist.status->done) {
    control_loops::wrist.status.FetchNextBlocking();
    LOG(DEBUG, "Got a new wrist status packet\n");
    if (ShouldExitAuto()) return;
  }
  LOG(INFO, "Done waiting for the wrist\n");
}
void WaitForIndex() {
  LOG(INFO, "Waiting for the indexer to be ready to intake\n");
  control_loops::index_loop.status.FetchLatest();

  while (!control_loops::index_loop.status.get()) {
    LOG(WARNING, "No previous index packet, trying to fetch again\n");
    control_loops::index_loop.status.FetchNextBlocking();
  }

  while (!control_loops::index_loop.status->ready_to_intake) {
    control_loops::index_loop.status.FetchNextBlocking();
    if (ShouldExitAuto()) return;
  }
  LOG(INFO, "Indexer ready to intake\n");
}

void WaitForAngle_Adjust() {
  LOG(INFO, "Waiting for the angle adjuster to finish\n");
  control_loops::angle_adjust.status.FetchLatest();

  while (!control_loops::angle_adjust.status.get()) {
    LOG(WARNING, "No previous angle adjust packet, trying to fetch again\n");
    control_loops::angle_adjust.status.FetchNextBlocking();
  }

  while (!control_loops::angle_adjust.status->done) {
    control_loops::angle_adjust.status.FetchNextBlocking();
    if (ShouldExitAuto()) return;
  }
  LOG(INFO, "Angle adjuster done\n");
}

void WaitForShooter() {
  LOG(INFO, "Waiting for the shooter to spin up\n");
  control_loops::shooter.status.FetchLatest();

  while (!control_loops::shooter.status.get()) {
    LOG(WARNING, "No previous shooteracket, trying to fetch again\n");
    control_loops::shooter.status.FetchNextBlocking();
  }

  while (!control_loops::shooter.status->ready) {
    control_loops::shooter.status.FetchNextBlocking();
    if (ShouldExitAuto()) return;
  }
  LOG(INFO, "Shooter ready to shoot\n");
}

void ShootNDiscs(int n) {
  LOG(INFO, "Waiting until %d discs have been shot\n", n);
  control_loops::index_loop.status.FetchLatest();

  while (!control_loops::index_loop.status.get()) {
    LOG(WARNING, "No previous index_loop packet, trying to fetch again\n");
    control_loops::index_loop.status.FetchNextBlocking();
  }

  int final_disc_count = control_loops::index_loop.status->shot_disc_count + n;
  LOG(DEBUG, "Disc count should be %d when done\n", final_disc_count);
  while (final_disc_count > control_loops::index_loop.status->shot_disc_count) {
    control_loops::index_loop.status.FetchNextBlocking();
    if (ShouldExitAuto()) return;
  }
  LOG(INFO, "Shot %d discs\n", n);
}

void StopDrivetrain() {
  LOG(INFO, "Stopping the drivetrain\n");
  control_loops::drivetrain.goal.MakeWithBuilder()
      .control_loop_driving(true)
      .left_goal(left_initial_position)
      .left_velocity_goal(0)
      .right_goal(right_initial_position)
      .right_velocity_goal(0)
      .quickturn(false)
      .Send();
}

void ResetDrivetrain() {
  LOG(INFO, "resetting the drivetrain\n");
  control_loops::drivetrain.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .highgear(false)
      .steering(0.0)
      .throttle(0.0)
      .Send();
}

void SetDriveGoal(double yoffset, double maximum_velocity = 1.5) {
  LOG(INFO, "Going to move %f\n", yoffset);

  // Measured conversion to get the distance right.
  ::aos::util::TrapezoidProfile profile(::aos::time::Time::InMS(10));
  ::Eigen::Matrix<double, 2, 1> driveTrainState;
  const double goal_velocity = 0.0;
  const double epsilon = 0.01;

  profile.set_maximum_acceleration(2.0);
  profile.set_maximum_velocity(maximum_velocity);
 
  while (true) {
    ::aos::time::PhasedLoop10MS(5000);      // wait until next 10ms tick
    driveTrainState = profile.Update(yoffset, goal_velocity);

    if (::std::abs(driveTrainState(0, 0) - yoffset) < epsilon) break;
    if (ShouldExitAuto()) return;

    LOG(DEBUG, "Driving left to %f, right to %f\n",
        driveTrainState(0, 0) + left_initial_position,
        driveTrainState(0, 0) + right_initial_position);
    control_loops::drivetrain.goal.MakeWithBuilder()
        .control_loop_driving(true)
        .highgear(false)
        .left_goal(driveTrainState(0, 0) + left_initial_position)
        .right_goal(driveTrainState(0, 0) + right_initial_position)
        .left_velocity_goal(driveTrainState(1, 0))
        .right_velocity_goal(driveTrainState(1, 0))
        .Send();
  }
  left_initial_position += yoffset;
  right_initial_position += yoffset;
  LOG(INFO, "Done moving\n");
}

// Drives forward while we can pick up discs up to max_distance (in meters).
void DriveForwardPickUp(double max_distance, double wrist_angle) {
  LOG(INFO, "going to pick up at a max distance of %f\n", max_distance);

  static const ::aos::time::Time kPeriod = ::aos::time::Time::InMS(10);
  ::aos::util::TrapezoidProfile profile(kPeriod);
  ::Eigen::Matrix<double, 2, 1> driveTrainState;
  const double goal_velocity = 0.0;
  const double epsilon = 0.01;
  static const double kMaximumAcceleration = 1.0;

  profile.set_maximum_acceleration(kMaximumAcceleration);
  profile.set_maximum_velocity(0.6);

  bool driving_back = false;
  const double kDestination = -0.20;

  while (true) {
    ::aos::time::PhasedLoop10MS(5000);      // wait until next 10ms tick
    driveTrainState = profile.Update(driving_back ? kDestination : max_distance,
                                     goal_velocity);

    if (ShouldExitAuto()) return;

    if (driving_back) {
      if (::std::abs(driveTrainState(0, 0)) < epsilon) break;
    } else if (::std::abs(driveTrainState(0, 0) - max_distance) < epsilon) {
      LOG(INFO, "went the max distance; driving back\n");
      driving_back = true;
      profile.set_maximum_velocity(2.5);
      SetWristGoal(wrist_angle);
    }

    if (control_loops::index_loop.status.FetchLatest()) {
      if (control_loops::index_loop.status->hopper_disc_count >= 4) {
        LOG(INFO, "done intaking; driving back\n");
        driving_back = true;
        profile.set_maximum_velocity(2.5);
        SetWristGoal(wrist_angle);
      }
    } else {
      LOG(WARNING, "getting index status failed\n");
    }

    LOG(DEBUG, "Driving left to %f, right to %f\n",
        driveTrainState(0, 0) + left_initial_position,
        driveTrainState(0, 0) + right_initial_position);
    control_loops::drivetrain.goal.MakeWithBuilder()
        .control_loop_driving(true)
        .highgear(false)
        .left_goal(driveTrainState(0, 0) + left_initial_position)
        .right_goal(driveTrainState(0, 0) + right_initial_position)
        .left_velocity_goal(driveTrainState(1, 0))
        .right_velocity_goal(driveTrainState(1, 0))
        .Send();
  }
  left_initial_position += kDestination;
  right_initial_position += kDestination;
  LOG(INFO, "Done moving\n");
}

void DriveSpin(double radians) {
  LOG(INFO, "going to spin %f\n", radians);

  ::aos::util::TrapezoidProfile profile(::aos::time::Time::InMS(10));
  ::Eigen::Matrix<double, 2, 1> driveTrainState;
  const double goal_velocity = 0.0;
  const double epsilon = 0.01;
  // in drivetrain "meters"
  const double kRobotWidth = 0.4544;

  profile.set_maximum_acceleration(1.5);
  profile.set_maximum_velocity(0.8);

  const double side_offset = kRobotWidth * radians / 2.0;

  while (true) {
    ::aos::time::PhasedLoop10MS(5000);      // wait until next 10ms tick
    driveTrainState = profile.Update(side_offset, goal_velocity);

    if (::std::abs(driveTrainState(0, 0) - side_offset) < epsilon) break;
    if (ShouldExitAuto()) return;

    LOG(DEBUG, "Driving left to %f, right to %f\n",
        left_initial_position - driveTrainState(0, 0),
        right_initial_position + driveTrainState(0, 0));
    control_loops::drivetrain.goal.MakeWithBuilder()
        .control_loop_driving(true)
        .highgear(false)
        .left_goal(left_initial_position - driveTrainState(0, 0))
        .right_goal(right_initial_position + driveTrainState(0, 0))
        .left_velocity_goal(-driveTrainState(1, 0))
        .right_velocity_goal(driveTrainState(1, 0))
        .Send();
  }
  left_initial_position -= side_offset;
  right_initial_position += side_offset;
  LOG(INFO, "Done moving\n");
}

// start with N discs in the indexer
void HandleAuto() {
  LOG(INFO, "Handling auto mode\n");

  double WRIST_UP;
  const double WRIST_DOWN = -0.580;
  const double WRIST_DOWN_TWO = WRIST_DOWN - 0.010;
  const double ANGLE_ONE = 0.556;
  const double ANGLE_TWO = 0.677;

  ResetIndex();
  SetWristGoal(1.0);  // wrist must calibrate itself on power-up
  SetAngle_AdjustGoal(ANGLE_TWO);  // make it still move a bit
  SetShooterVelocity(0.0);  // or else it keeps spinning from last time
  ResetDrivetrain();

  //::aos::time::SleepFor(::aos::time::Time::InSeconds(20));
  if (ShouldExitAuto()) return;
  
  ::aos::robot_state.FetchLatest();
  if (!::aos::robot_state.get() ||
      !constants::wrist_hall_effect_start_angle(&WRIST_UP)) {
    LOG(ERROR, "Constants not ready\n");
    return;
  }
  WRIST_UP -= 0.4;
  LOG(INFO, "Got constants\n");

  control_loops::drivetrain.position.FetchLatest();
  while (!control_loops::drivetrain.position.get()) {
    LOG(WARNING, "No previous drivetrain position packet, trying to fetch again\n");
    control_loops::drivetrain.position.FetchNextBlocking();
  }
  left_initial_position =
    control_loops::drivetrain.position->left_encoder;
  right_initial_position =
    control_loops::drivetrain.position->right_encoder;

  StopDrivetrain();

  SetWristGoal(WRIST_UP);    // wrist must calibrate itself on power-up
  SetAngle_AdjustGoal(ANGLE_ONE);
  SetShooterVelocity(380.0);
  WaitForIndexReset();
  if (ShouldExitAuto()) return;
  PreloadIndex();      // spin to top and put 1 disc into loader

  if (ShouldExitAuto()) return;
  WaitForWrist();
  if (ShouldExitAuto()) return;
  WaitForAngle_Adjust();
  ShootIndex();        // tilt up, shoot, repeat until empty
          // calls WaitForShooter
  ShootNDiscs(3);      // ShootNDiscs returns if ShouldExitAuto
  if (ShouldExitAuto()) return;

  StartIndex();        // take in up to 4 discs

  if (false) {
    const double kDistanceToCenterMeters = 3.11023;
    const double kMaxPickupDistance = 2.5;
    const double kTurnToCenterDegrees = 78.2;

    // Drive back to the center line.
    SetDriveGoal(-kDistanceToCenterMeters);
    if (ShouldExitAuto()) return;

    SetWristGoal(WRIST_DOWN);
    // Turn towards the center.
    DriveSpin(kTurnToCenterDegrees * M_PI / 180.0);
    if (ShouldExitAuto()) return;
    WaitForWrist();
    if (ShouldExitAuto()) return;

    // Pick up at most 4 discs and drive at most kMaxPickupDistance.
    DriveForwardPickUp(kMaxPickupDistance, WRIST_UP);

    SetWristGoal(WRIST_UP);
    DriveSpin(-kTurnToCenterDegrees * M_PI / 180.0);
    if (ShouldExitAuto()) return;
    // Drive back to where we were.
    SetDriveGoal(kDistanceToCenterMeters);
    if (ShouldExitAuto()) return;

    return;

    ShootNDiscs(4);
    if (ShouldExitAuto()) return;
  } else {
    // Delay to let the disc out of the shooter.
    ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.25));
    SetWristGoal(WRIST_DOWN);
    StartIndex();				// take in up to 4 discs

    if (ShouldExitAuto()) return;
    WaitForWrist();			// wrist must be down before moving
    ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.25));
    SetAngle_AdjustGoal(ANGLE_TWO);
    SetShooterVelocity(375.0);

    if (ShouldExitAuto()) return; 
    WaitForIndex();			// ready to pick up discs

    static const double kDriveDistance = 2.8;
    static const double kFirstDrive = 0.27;
    static const double kSecondShootDistance = 2.0;
    SetDriveGoal(kFirstDrive, 0.6);
    SetWristGoal(WRIST_DOWN_TWO);
    SetDriveGoal(kDriveDistance - kFirstDrive, 2.0);
    if (ShouldExitAuto()) return;

    ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.5));
    SetDriveGoal(kSecondShootDistance - kDriveDistance, 2.0);
    PreloadIndex();

    if (ShouldExitAuto()) return;
    WaitForAngle_Adjust();
    if (ShouldExitAuto()) return;
    ShootIndex();	
    if (ShouldExitAuto()) return;
  }
}

}  // namespace autonomous
}  // namespace frc971
