#include "stdio.h"

#include "aos/common/control_loop/Timing.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/logging/logging.h"
#include "aos/common/network/team_number.h"

#include "frc971/autonomous/auto.q.h"
#include "frc971/constants.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"

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

void HandleAuto() {
  LOG(INFO, "Handling auto mode\n");

  ResetDrivetrain();

  if (ShouldExitAuto()) return;
  
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
}

}  // namespace autonomous
}  // namespace frc971
