#include "y2014/control_loops/drivetrain/drivetrain.h"

#include <stdio.h>
#include <sched.h>
#include <cmath>
#include <memory>
#include "Eigen/Dense"

#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/logging/matrix_logging.h"

#include "y2014/constants.h"
#include "y2014/control_loops/drivetrain/drivetrain.q.h"
#include "y2014/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2014/control_loops/drivetrain/polydrivetrain.h"
#include "y2014/control_loops/drivetrain/ssdrivetrain.h"
#include "frc971/queues/gyro.q.h"
#include "frc971/shifter_hall_effect.h"

// A consistent way to mark code that goes away without shifters. It's still
// here because we will have shifters again in the future.
#define HAVE_SHIFTERS 1

using frc971::sensors::gyro_reading;

namespace y2014 {
namespace control_loops {
namespace drivetrain {

void DrivetrainLoop::RunIteration(
    const ::frc971::control_loops::DrivetrainQueue::Goal *goal,
    const ::frc971::control_loops::DrivetrainQueue::Position *position,
    ::frc971::control_loops::DrivetrainQueue::Output *output,
    ::frc971::control_loops::DrivetrainQueue::Status *status) {
  bool bad_pos = false;
  if (position == nullptr) {
    LOG_INTERVAL(no_position_);
    bad_pos = true;
  }
  no_position_.Print();

  bool control_loop_driving = false;
  if (goal) {
    double wheel = goal->steering;
    double throttle = goal->throttle;
    bool quickturn = goal->quickturn;
#if HAVE_SHIFTERS
    bool highgear = goal->highgear;
#endif

    control_loop_driving = goal->control_loop_driving;
    double left_goal = goal->left_goal;
    double right_goal = goal->right_goal;

    dt_closedloop_.SetGoal(left_goal, goal->left_velocity_goal, right_goal,
                           goal->right_velocity_goal);
#if HAVE_SHIFTERS
    dt_openloop_.SetGoal(wheel, throttle, quickturn, highgear);
#else
    dt_openloop_.SetGoal(wheel, throttle, quickturn, false);
#endif
  }

  if (!bad_pos) {
    const double left_encoder = position->left_encoder;
    const double right_encoder = position->right_encoder;
    if (gyro_reading.FetchLatest()) {
      LOG_STRUCT(DEBUG, "using", *gyro_reading.get());
      dt_closedloop_.SetPosition(left_encoder, right_encoder,
                                 gyro_reading->angle);
    } else {
      dt_closedloop_.SetRawPosition(left_encoder, right_encoder);
    }
  }
  dt_openloop_.SetPosition(position);
  dt_openloop_.Update();

  if (control_loop_driving) {
    dt_closedloop_.Update(output == NULL, true);
    dt_closedloop_.SendMotors(output);
  } else {
    dt_openloop_.SendMotors(output);
    if (output) {
      dt_closedloop_.SetExternalMotors(output->left_voltage,
                                       output->right_voltage);
    }
    dt_closedloop_.Update(output == NULL, false);
  }

  // set the output status of the control loop state
  if (status) {
    bool done = false;
    if (goal) {
      done = ((::std::abs(goal->left_goal -
                          dt_closedloop_.GetEstimatedLeftEncoder()) <
               constants::GetValues().drivetrain_done_distance) &&
              (::std::abs(goal->right_goal -
                          dt_closedloop_.GetEstimatedRightEncoder()) <
               constants::GetValues().drivetrain_done_distance));
    }
    status->is_done = done;
    status->robot_speed = dt_closedloop_.GetEstimatedRobotSpeed();
    status->filtered_left_position = dt_closedloop_.GetEstimatedLeftEncoder();
    status->filtered_right_position = dt_closedloop_.GetEstimatedRightEncoder();

    status->filtered_left_velocity = dt_closedloop_.loop().X_hat(1, 0);
    status->filtered_right_velocity = dt_closedloop_.loop().X_hat(3, 0);
    status->output_was_capped = dt_closedloop_.OutputWasCapped();
    status->uncapped_left_voltage = dt_closedloop_.loop().U_uncapped(0, 0);
    status->uncapped_right_voltage = dt_closedloop_.loop().U_uncapped(1, 0);
  }
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2014
