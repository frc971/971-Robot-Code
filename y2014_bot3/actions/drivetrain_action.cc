#include <functional>
#include <numeric>

#include <Eigen/Dense>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/commonmath.h"

#include "bot3/actions/drivetrain_action.h"
#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "bot3/control_loops/drivetrain/drivetrain_constants.h"
#include "bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

namespace bot3 {
namespace actions {

DrivetrainAction::DrivetrainAction(::frc971::actions::DrivetrainActionQueueGroup* s)
    : ::frc971::actions::ActionBase
        <::frc971::actions::DrivetrainActionQueueGroup>(s) {}

void DrivetrainAction::RunAction() {
  static const auto K = control_loops::MakeDrivetrainLoop().K();

  const double yoffset = action_q_->goal->y_offset;
  const double turn_offset =
      action_q_->goal->theta_offset * control_loops::kBot3TurnWidth / 2.0;
  LOG(INFO, "Going to move %f and turn %f\n", yoffset, turn_offset);

  // Measured conversion to get the distance right.
  ::aos::util::TrapezoidProfile profile(::aos::time::Time::InMS(10));
  ::aos::util::TrapezoidProfile turn_profile(::aos::time::Time::InMS(10));
  const double goal_velocity = 0.0;
  const double epsilon = 0.01;
  ::Eigen::Matrix<double, 2, 1> left_goal_state, right_goal_state;

  profile.set_maximum_acceleration(action_q_->goal->maximum_acceleration);
  profile.set_maximum_velocity(action_q_->goal->maximum_velocity);
  turn_profile.set_maximum_acceleration(
      10.0 * control_loops::kBot3TurnWidth / 2.0);
  turn_profile.set_maximum_velocity(3.0 * control_loops::kBot3TurnWidth / 2.0);

  while (true) {
    // wait until next 10ms tick
    ::aos::time::PhasedLoop10MS(5000);

    control_loops::drivetrain.status.FetchLatest();
    if (control_loops::drivetrain.status.get()) {
      const auto &status = *control_loops::drivetrain.status;
      if (::std::abs(status.uncapped_left_voltage -
                     status.uncapped_right_voltage) >
          24) {
        LOG(DEBUG, "spinning in place\n");
        // They're more than 24V apart, so stop moving forwards and let it deal
        // with spinning first.
        profile.SetGoal(
            (status.filtered_left_position + status.filtered_right_position) /
            2.0);
      } else {
        static const double divisor = K(0, 0) + K(0, 2);
        double dx_left, dx_right;
        if (status.uncapped_left_voltage > 12.0) {
          dx_left = (status.uncapped_left_voltage - 12.0) / divisor;
        } else if (status.uncapped_left_voltage < -12.0) {
          dx_left = (status.uncapped_left_voltage + 12.0) / divisor;
        } else {
          dx_left = 0;
        }
        if (status.uncapped_right_voltage > 12.0) {
          dx_right = (status.uncapped_right_voltage - 12.0) / divisor;
        } else if (status.uncapped_right_voltage < -12.0) {
          dx_right = (status.uncapped_right_voltage + 12.0) / divisor;
        } else {
          dx_right = 0;
        }
        double dx;
        if (dx_left == 0 && dx_right == 0) {
          dx = 0;
        } else if (dx_left != 0 && dx_right != 0 &&
                   ::aos::sign(dx_left) != ::aos::sign(dx_right)) {
          // Both saturating in opposite directions. Don't do anything.
          dx = 0;
        } else if (::std::abs(dx_left) > ::std::abs(dx_right)) {
          dx = dx_left;
        } else {
          dx = dx_right;
        }
        if (dx != 0) {
          LOG(DEBUG, "adjusting goal by %f\n", dx);
          profile.MoveGoal(-dx);
        }
      }
    } else {
      // If we ever get here, that's bad and we should just give up
      LOG(FATAL, "no drivetrain status!\n");
    }

    const auto drive_profile_goal_state =
        profile.Update(yoffset, goal_velocity);
    const auto turn_profile_goal_state = turn_profile.Update(turn_offset, 0.0);
    left_goal_state = drive_profile_goal_state - turn_profile_goal_state;
    right_goal_state = drive_profile_goal_state + turn_profile_goal_state;

    if (::std::abs(drive_profile_goal_state(0, 0) - yoffset) < epsilon &&
        ::std::abs(turn_profile_goal_state(0, 0) - turn_offset) < epsilon) {
      break;
    }
    if (ShouldCancel()) return;

    LOG(DEBUG, "Driving left to %f, right to %f\n",
        left_goal_state(0, 0) + action_q_->goal->left_initial_position,
        right_goal_state(0, 0) + action_q_->goal->right_initial_position);
    control_loops::drivetrain.goal.MakeWithBuilder()
        .control_loop_driving(true)
        .highgear(false)
        .left_goal(left_goal_state(0, 0) + action_q_->goal->left_initial_position)
        .right_goal(right_goal_state(0, 0) + action_q_->goal->right_initial_position)
        .left_velocity_goal(left_goal_state(1, 0))
        .right_velocity_goal(right_goal_state(1, 0))
        .Send();
  }
  if (ShouldCancel()) return;
  control_loops::drivetrain.status.FetchLatest();
  while (!control_loops::drivetrain.status.get()) {
    LOG(WARNING,
        "No previous drivetrain status packet, trying to fetch again\n");
    control_loops::drivetrain.status.FetchNextBlocking();
    if (ShouldCancel()) return;
  }
  while (true) {
    if (ShouldCancel()) return;
    const double kPositionThreshold = 0.05;

    const double left_error = ::std::abs(
        control_loops::drivetrain.status->filtered_left_position -
        (left_goal_state(0, 0) + action_q_->goal->left_initial_position));
    const double right_error = ::std::abs(
        control_loops::drivetrain.status->filtered_right_position -
        (right_goal_state(0, 0) + action_q_->goal->right_initial_position));
    const double velocity_error =
        ::std::abs(control_loops::drivetrain.status->robot_speed);
    if (left_error < kPositionThreshold && right_error < kPositionThreshold &&
        velocity_error < 0.2) {
      break;
    } else {
      LOG(DEBUG, "Drivetrain error is %f, %f, %f\n", left_error, right_error,
          velocity_error);
    }
    control_loops::drivetrain.status.FetchNextBlocking();
  }
  LOG(INFO, "Done moving\n");
}

::std::unique_ptr<::frc971::TypedAction< ::frc971::actions::DrivetrainActionQueueGroup>>
MakeDrivetrainAction() {
  return ::std::unique_ptr<
      ::frc971::TypedAction< ::frc971::actions::DrivetrainActionQueueGroup>>(
      new ::frc971::TypedAction< ::frc971::actions::DrivetrainActionQueueGroup>(
          &::frc971::actions::drivetrain_action));
}

}  // namespace actions
}  // namespace bot3
