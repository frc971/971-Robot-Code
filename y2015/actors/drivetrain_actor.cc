#include "y2015/actors/drivetrain_actor.h"

#include <functional>
#include <numeric>

#include <Eigen/Dense>

#include "aos/common/util/phased_loop.h"
#include "aos/common/logging/logging.h"
#include "aos/common/util/trapezoid_profile.h"
#include "aos/common/commonmath.h"
#include "aos/common/time.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "y2015/actors/drivetrain_actor.h"
#include "y2015/constants.h"

namespace y2015 {
namespace actors {

namespace chrono = ::std::chrono;
using ::frc971::control_loops::drivetrain_queue;

DrivetrainActor::DrivetrainActor(actors::DrivetrainActionQueueGroup* s)
    : aos::common::actions::ActorBase<actors::DrivetrainActionQueueGroup>(s) {}

bool DrivetrainActor::RunAction(const actors::DrivetrainActionParams &params) {
  static const auto K =
      constants::GetValues().make_drivetrain_loop().controller().K();

  const double yoffset = params.y_offset;
  const double turn_offset =
      params.theta_offset * constants::GetValues().turn_width / 2.0;
  LOG(INFO, "Going to move %f and turn %f\n", yoffset, turn_offset);

  // Measured conversion to get the distance right.
  ::aos::util::TrapezoidProfile profile(chrono::milliseconds(5));
  ::aos::util::TrapezoidProfile turn_profile(chrono::milliseconds(5));
  const double goal_velocity = 0.0;
  const double epsilon = 0.01;
  ::Eigen::Matrix<double, 2, 1> left_goal_state, right_goal_state;

  profile.set_maximum_acceleration(params.maximum_acceleration);
  profile.set_maximum_velocity(params.maximum_velocity);
  turn_profile.set_maximum_acceleration(params.maximum_turn_acceleration *
                                        constants::GetValues().turn_width /
                                        2.0);
  turn_profile.set_maximum_velocity(params.maximum_turn_velocity *
                                    constants::GetValues().turn_width / 2.0);

  ::aos::time::PhasedLoop phased_loop(::std::chrono::milliseconds(5),
                                      ::std::chrono::milliseconds(5) / 2);
  while (true) {
    phased_loop.SleepUntilNext();

    drivetrain_queue.status.FetchLatest();
    if (drivetrain_queue.status.get()) {
      const auto& status = *drivetrain_queue.status;
      if (::std::abs(status.uncapped_left_voltage -
                     status.uncapped_right_voltage) > 24) {
        LOG(DEBUG, "spinning in place\n");
        // They're more than 24V apart, so stop moving forwards and let it deal
        // with spinning first.
        profile.SetGoal(
            (status.estimated_left_position + status.estimated_right_position -
             params.left_initial_position - params.right_initial_position) /
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
          LOG(DEBUG, "Saturating opposite ways, not adjusting\n");
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
      LOG(ERROR, "no drivetrain status!\n");
      return false;
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

    if (ShouldCancel()) return true;

    LOG(DEBUG, "Driving left to %f, right to %f\n",
        left_goal_state(0, 0) + params.left_initial_position,
        right_goal_state(0, 0) + params.right_initial_position);
    drivetrain_queue.goal.MakeWithBuilder()
        .control_loop_driving(true)
        //.highgear(false)
        .left_goal(left_goal_state(0, 0) + params.left_initial_position)
        .right_goal(right_goal_state(0, 0) + params.right_initial_position)
        .left_velocity_goal(left_goal_state(1, 0))
        .right_velocity_goal(right_goal_state(1, 0))
        .Send();
  }
  if (ShouldCancel()) return true;
  drivetrain_queue.status.FetchLatest();
  while (!drivetrain_queue.status.get()) {
    LOG(WARNING,
        "No previous drivetrain status packet, trying to fetch again\n");
    drivetrain_queue.status.FetchNextBlocking();
    if (ShouldCancel()) return true;
  }
  while (true) {
    if (ShouldCancel()) return true;
    const double kPositionThreshold = 0.05;

    const double left_error =
        ::std::abs(drivetrain_queue.status->estimated_left_position -
                   (left_goal_state(0, 0) + params.left_initial_position));
    const double right_error =
        ::std::abs(drivetrain_queue.status->estimated_right_position -
                   (right_goal_state(0, 0) + params.right_initial_position));
    const double velocity_error =
        ::std::abs(drivetrain_queue.status->robot_speed);
    if (left_error < kPositionThreshold && right_error < kPositionThreshold &&
        velocity_error < 0.2) {
      break;
    } else {
      LOG(DEBUG, "Drivetrain error is %f, %f, %f\n", left_error, right_error,
          velocity_error);
    }
    drivetrain_queue.status.FetchNextBlocking();
  }
  LOG(INFO, "Done moving\n");
  return true;
}

::std::unique_ptr<DrivetrainAction> MakeDrivetrainAction(
    const ::y2015::actors::DrivetrainActionParams& params) {
  return ::std::unique_ptr<DrivetrainAction>(
      new DrivetrainAction(&::y2015::actors::drivetrain_action, params));
}

}  // namespace actors
}  // namespace y2015
