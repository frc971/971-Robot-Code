#include <stdio.h>

#include <memory>

#include "aos/common/util/phased_loop.h"
#include "aos/common/time.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"

#include "frc971/autonomous/auto.q.h"
#include "frc971/constants.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/actions/action_client.h"
#include "frc971/actions/drivetrain_action.h"

using ::aos::time::Time;

namespace frc971 {
namespace autonomous {

namespace time = ::aos::time;

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
      //.highgear(false)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(left_initial_position)
      .left_velocity_goal(0)
      .right_goal(right_initial_position)
      .right_velocity_goal(0)
      .Send();
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
        //.highgear(false)
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

void WaitUntilDoneOrCanceled(Action *action) {
  while (true) {
    // Poll the running bit and auto done bits.
    ::aos::time::PhasedLoop10MS(5000);
    if (!action->Running() || ShouldExitAuto()) {
      return;
    }
  }
}

::std::unique_ptr<TypedAction< ::frc971::actions::DrivetrainActionQueueGroup>>
SetDriveGoal(double distance, bool slow_acceleration,
             double maximum_velocity = 1.7, double theta = 0) {
  LOG(INFO, "Driving to %f\n", distance);
  auto drivetrain_action = actions::MakeDrivetrainAction();
  drivetrain_action->GetGoal()->left_initial_position = left_initial_position;
  drivetrain_action->GetGoal()->right_initial_position = right_initial_position;
  drivetrain_action->GetGoal()->y_offset = distance;
  drivetrain_action->GetGoal()->theta_offset = theta;
  drivetrain_action->GetGoal()->maximum_velocity = maximum_velocity;
  drivetrain_action->GetGoal()->maximum_acceleration =
      slow_acceleration ? 2.5 : 3.0;
  drivetrain_action->Start();
  left_initial_position +=
      distance - theta * constants::GetValues().turn_width / 2.0;
  right_initial_position +=
      distance + theta * constants::GetValues().turn_width / 2.0;
  return ::std::move(drivetrain_action);
}

void InitializeEncoders() {
  control_loops::drivetrain.status.FetchLatest();
  while (!control_loops::drivetrain.status.get()) {
    LOG(WARNING,
        "No previous drivetrain position packet, trying to fetch again\n");
    control_loops::drivetrain.status.FetchNextBlocking();
  }
  left_initial_position =
    control_loops::drivetrain.status->filtered_left_position;
  right_initial_position =
    control_loops::drivetrain.status->filtered_right_position;

}

void HandleAuto() {
  ::aos::time::Time start_time = ::aos::time::Time::Now();
  LOG(INFO, "Handling auto mode at %f\n", start_time.ToSeconds());

  ResetDrivetrain();

  if (ShouldExitAuto()) return;
  InitializeEncoders();
}

}  // namespace autonomous
}  // namespace frc971
