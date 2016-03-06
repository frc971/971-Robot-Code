#include "y2016/autonomous/auto.h"

#include <stdio.h>

#include <memory>

#include "aos/common/actions/actions.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/time.h"
#include "aos/common/util/phased_loop.h"
#include "aos/common/util/trapezoid_profile.h"

#include "frc971/autonomous/auto.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2016/actors/drivetrain_actor.h"
#include "y2016/constants.h"
#include "y2016/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2016/queues/profile_params.q.h"

using ::aos::time::Time;

namespace y2016 {
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

void ResetDrivetrain() {
  LOG(INFO, "resetting the drivetrain\n");
  ::frc971::control_loops::drivetrain_queue.goal.MakeWithBuilder()
      .control_loop_driving(false)
      .highgear(true)
      .steering(0.0)
      .throttle(0.0)
      .left_goal(left_initial_position)
      .left_velocity_goal(0)
      .right_goal(right_initial_position)
      .right_velocity_goal(0)
      .Send();
}

void WaitUntilDoneOrCanceled(
    ::std::unique_ptr<aos::common::actions::Action> action) {
  if (!action) {
    LOG(ERROR, "No action, not waiting\n");
    return;
  }
  while (true) {
    // Poll the running bit and auto done bits.
    ::aos::time::PhasedLoopXMS(10, 5000);
    if (!action->Running() || ShouldExitAuto()) {
      return;
    }
  }
}

const ProfileParams kFastDrive = {3.0, 2.5};
const ProfileParams kSlowDrive = {2.5, 2.5};
const ProfileParams kFastTurn = {3.0, 10.0};

::std::unique_ptr<::y2016::actors::DrivetrainAction> SetDriveGoal(
    double distance, const ProfileParams drive_params, double theta = 0,
    const ProfileParams &turn_params = kFastTurn) {
  LOG(INFO, "Driving to %f\n", distance);

  ::y2016::actors::DrivetrainActionParams params;
  params.left_initial_position = left_initial_position;
  params.right_initial_position = right_initial_position;
  params.y_offset = distance;
  params.theta_offset = theta;
  params.maximum_turn_acceleration = turn_params.acceleration;
  params.maximum_turn_velocity = turn_params.velocity;
  params.maximum_velocity = drive_params.velocity;
  params.maximum_acceleration = drive_params.acceleration;
  auto drivetrain_action = actors::MakeDrivetrainAction(params);
  drivetrain_action->Start();
  left_initial_position +=
      distance - theta * control_loops::drivetrain::kRobotRadius;
  right_initial_position +=
      distance + theta * control_loops::drivetrain::kRobotRadius;
  return ::std::move(drivetrain_action);
}

void InitializeEncoders() {
  ::frc971::control_loops::drivetrain_queue.status.FetchAnother();
  left_initial_position =
      ::frc971::control_loops::drivetrain_queue.status->estimated_left_position;
  right_initial_position =
      ::frc971::control_loops::drivetrain_queue.status->estimated_right_position;
}

void HandleAuto() {
  LOG(INFO, "Handling auto mode\n");

  ResetDrivetrain();
  InitializeEncoders();
}

}  // namespace autonomous
}  // namespace y2016
