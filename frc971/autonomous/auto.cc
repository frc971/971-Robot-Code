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
					// Index_loop has no FetchNextBlocking
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
      .highgear(false)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(left_initial_position)
      .right_goal(right_initial_position)
      .quickturn(false)
      .Send();
}

void SetDriveGoal(double yoffset) {
  LOG(INFO, "Going to move %f\n", yoffset);

  // Measured conversion to get the distance right.
  yoffset *= 0.73;
  LOG(INFO, "Going to move an adjusted %f\n", yoffset);
  ::aos::util::TrapezoidProfile profile(::aos::time::Time::InMS(10));
  ::Eigen::Matrix<double, 2, 1> driveTrainState;
  const double goal_velocity = 0.0;
  const double epsilon = 0.01;

  profile.set_maximum_acceleration(1);
  profile.set_maximum_velocity(0.6);
 
  while (true) {
    ::aos::time::PhasedLoop10MS(5000);			// wait until next 10ms tick
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

void DriveSpin(double radians) {
  LOG(INFO, "going to spin %f\n", radians);

  ::aos::util::TrapezoidProfile profile(::aos::time::Time::InMS(10));
  ::Eigen::Matrix<double, 2, 1> driveTrainState;
  const double goal_velocity = 0.0;
  const double epsilon = 0.01;
  // in meters
  const double kRobotWidth = 0.4544;

  profile.set_maximum_acceleration(1);
  profile.set_maximum_velocity(0.6);

  const double side_offset = kRobotWidth * radians / 2.0;

  while (true) {
    ::aos::time::PhasedLoop10MS(5000);			// wait until next 10ms tick
    driveTrainState = profile.Update(side_offset, goal_velocity);

    if (::std::abs(driveTrainState(0, 0) - side_offset) < epsilon) break;
    if (ShouldExitAuto()) return;

    LOG(DEBUG, "Driving left to %f, right to %f\n",
        left_initial_position + driveTrainState(0, 0),
        right_initial_position - driveTrainState(0, 0));
    control_loops::drivetrain.goal.MakeWithBuilder()
        .control_loop_driving(true)
        .highgear(false)
        .left_goal(left_initial_position + driveTrainState(0, 0))
        .right_goal(right_initial_position - driveTrainState(0, 0))
        .left_velocity_goal(driveTrainState(1, 0))
        .right_velocity_goal(-driveTrainState(1, 0))
        .Send();
  }
  left_initial_position += side_offset;
  right_initial_position -= side_offset;
  LOG(INFO, "Done moving\n");
}

// start with N discs in the indexer
void HandleAuto() {
  LOG(INFO, "Handling auto mode\n");
  double WRIST_UP;
  
  ::aos::robot_state.FetchLatest();
  if (!::aos::robot_state.get() ||
      !constants::wrist_hall_effect_start_angle(&WRIST_UP)) {
    LOG(ERROR, "Constants not ready\n");
    return;
  }
  WRIST_UP -= 0.4;
  LOG(INFO, "Got constants\n");
  const double WRIST_DOWN = -0.633;
  const double ANGLE_ONE = 0.5101;
  const double ANGLE_TWO = 0.685;

  control_loops::drivetrain.position.FetchLatest();
  while (!control_loops::drivetrain.position.get()) {
    LOG(WARNING, "No previous drivetrain position packet, trying to fetch again\n");
    control_loops::drivetrain.position.FetchNextBlocking();
  }
  left_initial_position =
    control_loops::drivetrain.position->left_encoder;
  right_initial_position =
    control_loops::drivetrain.position->right_encoder;

  ResetIndex();
  StopDrivetrain();

  SetWristGoal(WRIST_UP);		// wrist must calibrate itself on power-up
  SetAngle_AdjustGoal(ANGLE_ONE);
  SetShooterVelocity(410.0);
  WaitForIndexReset();

  // Wait to get some more air pressure in the thing.
  ::aos::time::SleepFor(::aos::time::Time::InSeconds(5.0));
  
  PreloadIndex();			// spin to top and put 1 disc into loader

  if (ShouldExitAuto()) return;
  WaitForWrist();
  if (ShouldExitAuto()) return;
  WaitForAngle_Adjust();
  ShootIndex();				// tilt up, shoot, repeat until empty
					// calls WaitForShooter
  ShootNDiscs(3);			// ShootNDiscs returns if ShouldExitAuto
  if (ShouldExitAuto()) return;
  ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.25));

  return;

  SetWristGoal(WRIST_DOWN);
  SetAngle_AdjustGoal(ANGLE_TWO);
  SetShooterVelocity(375.0);
  StartIndex();				// take in up to 4 discs

  if (ShouldExitAuto()) return;
  WaitForWrist();			// wrist must be down before moving
  ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.25));

  if (ShouldExitAuto()) return; 
  WaitForIndex();			// ready to pick up discs
 
  SetDriveGoal(3.5);
  //SetDriveGoal(0.6);
  //::aos::time::SleepFor(::aos::time::Time::InSeconds(0.25));
  //SetDriveGoal(2.9);
  if (ShouldExitAuto()) return;

  PreloadIndex();
  ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.4));
  SetDriveGoal(-1.3);

  if (ShouldExitAuto()) return;
  WaitForAngle_Adjust();
  ShootIndex();	
}

}  // namespace autonomous
}  // namespace frc971
