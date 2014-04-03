#include <functional>
#include <numeric>

#include <Eigen/Dense>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/trapezoid_profile.h"

#include "frc971/actions/drivetrain_action.h"
#include "frc971/constants.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"

namespace frc971 {
namespace actions {

DrivetrainAction::DrivetrainAction(actions::DrivetrainActionQueueGroup* s)
    : actions::ActionBase<actions::DrivetrainActionQueueGroup>(s) {}

void DrivetrainAction::RunAction() {
  const double yoffset = action_q_->goal->y_offset;
  LOG(INFO, "Going to move %f\n", yoffset);

  // Measured conversion to get the distance right.
  ::aos::util::TrapezoidProfile profile(::aos::time::Time::InMS(10));
  ::Eigen::Matrix<double, 2, 1> profile_goal_state;
  const double goal_velocity = 0.0;
  const double epsilon = 0.01;

  profile.set_maximum_acceleration(2.2);
  profile.set_maximum_velocity(action_q_->goal->maximum_velocity);

  while (true) {
    // wait until next 10ms tick
    ::aos::time::PhasedLoop10MS(5000);
    profile_goal_state = profile.Update(yoffset, goal_velocity);

    if (::std::abs(profile_goal_state(0, 0) - yoffset) < epsilon) break;
    if (ShouldCancel()) return;

    LOG(DEBUG, "Driving left to %f, right to %f\n",
        profile_goal_state(0, 0) + action_q_->goal->left_initial_position,
        profile_goal_state(0, 0) + action_q_->goal->right_initial_position);
    control_loops::drivetrain.goal.MakeWithBuilder()
        .control_loop_driving(true)
        .highgear(false)
        .left_goal(profile_goal_state(0, 0) + action_q_->goal->left_initial_position)
        .right_goal(profile_goal_state(0, 0) + action_q_->goal->right_initial_position)
        .left_velocity_goal(profile_goal_state(1, 0))
        .right_velocity_goal(profile_goal_state(1, 0))
        .Send();
  }
  LOG(INFO, "Done moving\n");
}

::std::unique_ptr<TypedAction< ::frc971::actions::DrivetrainActionQueueGroup>>
MakeDrivetrainAction() {
  return ::std::unique_ptr<
      TypedAction< ::frc971::actions::DrivetrainActionQueueGroup>>(
      new TypedAction< ::frc971::actions::DrivetrainActionQueueGroup>(
          &::frc971::actions::drivetrain_action));
}

}  // namespace actions
}  // namespace frc971
