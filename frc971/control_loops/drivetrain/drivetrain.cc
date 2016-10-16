#include "frc971/control_loops/drivetrain/drivetrain.h"

#include <stdio.h>
#include <sched.h>
#include <cmath>
#include <memory>
#include "Eigen/Dense"

#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/logging/matrix_logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/polydrivetrain.h"
#include "frc971/control_loops/drivetrain/ssdrivetrain.h"
#include "frc971/control_loops/drivetrain/down_estimator.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/queues/gyro.q.h"
#include "frc971/shifter_hall_effect.h"
#include "frc971/wpilib/imu.q.h"

using frc971::sensors::gyro_reading;

namespace frc971 {
namespace control_loops {
namespace drivetrain {

DrivetrainLoop::DrivetrainLoop(
    const DrivetrainConfig &dt_config,
    ::frc971::control_loops::DrivetrainQueue *my_drivetrain)
    : aos::controls::ControlLoop<::frc971::control_loops::DrivetrainQueue>(
          my_drivetrain),
      dt_config_(dt_config),
      kf_(dt_config_.make_kf_drivetrain_loop()),
      dt_openloop_(dt_config_, &kf_),
      dt_closedloop_(dt_config_, &kf_, &integrated_kf_heading_),
      down_estimator_(MakeDownEstimatorLoop()),
      left_gear_(dt_config_.default_high_gear ? Gear::HIGH : Gear::LOW),
      right_gear_(dt_config_.default_high_gear ? Gear::HIGH : Gear::LOW),
      left_high_requested_(dt_config_.default_high_gear),
      right_high_requested_(dt_config_.default_high_gear) {
  ::aos::controls::HPolytope<0>::Init();
  down_U_.setZero();
}

int DrivetrainLoop::ControllerIndexFromGears() {
  if (MaybeHigh(left_gear_)) {
    if (MaybeHigh(right_gear_)) {
      return 3;
    } else {
      return 2;
    }
  } else {
    if (MaybeHigh(right_gear_)) {
      return 1;
    } else {
      return 0;
    }
  }
}

Gear ComputeGear(double shifter_position,
                 const constants::ShifterHallEffect &shifter_config,
                 bool high_requested) {
  if (shifter_position < shifter_config.clear_low) {
    return Gear::LOW;
  } else if (shifter_position > shifter_config.clear_high) {
    return Gear::HIGH;
  } else {
    if (high_requested) {
      return Gear::SHIFTING_UP;
    } else {
      return Gear::SHIFTING_DOWN;
    }
  }
}

void DrivetrainLoop::RunIteration(
    const ::frc971::control_loops::DrivetrainQueue::Goal *goal,
    const ::frc971::control_loops::DrivetrainQueue::Position *position,
    ::frc971::control_loops::DrivetrainQueue::Output *output,
    ::frc971::control_loops::DrivetrainQueue::Status *status) {
  if (!has_been_enabled_ && output) {
    has_been_enabled_ = true;
    down_estimator_.mutable_X_hat(1, 0) = 0.0;
  }

  // TODO(austin): Put gear detection logic here.
  switch (dt_config_.shifter_type) {
    case ShifterType::SIMPLE_SHIFTER:
      // Force the right controller for simple shifters since we assume that
      // gear switching is instantaneous.
      if (left_high_requested_) {
        left_gear_ = Gear::HIGH;
      } else {
        left_gear_ = Gear::LOW;
      }
      if (right_high_requested_) {
        right_gear_ = Gear::HIGH;
      } else {
        right_gear_ = Gear::LOW;
      }
      break;
    case ShifterType::HALL_EFFECT_SHIFTER:
      left_gear_ = ComputeGear(position->left_shifter_position,
                               dt_config_.left_drive, left_high_requested_);
      right_gear_ = ComputeGear(position->right_shifter_position,
                                dt_config_.right_drive, right_high_requested_);
      break;
    case ShifterType::NO_SHIFTER:
      break;
  }

  kf_.set_controller_index(ControllerIndexFromGears());
  {
    GearLogging gear_logging;
    gear_logging.left_state = static_cast<uint32_t>(left_gear_);
    gear_logging.right_state = static_cast<uint32_t>(right_gear_);
    gear_logging.left_loop_high = MaybeHigh(left_gear_);
    gear_logging.right_loop_high = MaybeHigh(right_gear_);
    gear_logging.controller_index = kf_.controller_index();
    LOG_STRUCT(DEBUG, "state", gear_logging);
  }

  if (::frc971::imu_values.FetchLatest()) {
    const double rate = -::frc971::imu_values->gyro_y;
    const double accel_squared = ::frc971::imu_values->accelerometer_x *
                                     ::frc971::imu_values->accelerometer_x +
                                 ::frc971::imu_values->accelerometer_y *
                                     ::frc971::imu_values->accelerometer_y +
                                 ::frc971::imu_values->accelerometer_z *
                                     ::frc971::imu_values->accelerometer_z;
    const double angle = ::std::atan2(::frc971::imu_values->accelerometer_x,
                                      ::frc971::imu_values->accelerometer_z) +
                         0.008;
    if (accel_squared > 1.03 || accel_squared < 0.97) {
      LOG(DEBUG, "New IMU value, rejecting reading\n");
    } else {
      // -y is our gyro.
      // z accel is down
      // x accel is the front of the robot pointed down.
      Eigen::Matrix<double, 1, 1> Y;
      Y(0, 0) = angle;
      down_estimator_.Correct(Y);
    }

    LOG(DEBUG,
        "New IMU value from ADIS16448, rate is %f, angle %f, fused %f, bias "
        "%f\n",
        rate, angle, down_estimator_.X_hat(0, 0), down_estimator_.X_hat(1, 0));
    down_U_(0, 0) = rate;
  }
  down_estimator_.UpdateObserver(down_U_);

  // TODO(austin): Signal the current gear to both loops.

  if (gyro_reading.FetchLatest()) {
    LOG_STRUCT(DEBUG, "using", *gyro_reading.get());
    last_gyro_heading_ = gyro_reading->angle;
    last_gyro_rate_ = gyro_reading->velocity;
  }

  {
    Eigen::Matrix<double, 3, 1> Y;
    Y << position->left_encoder, position->right_encoder, last_gyro_rate_;
    kf_.Correct(Y);
    integrated_kf_heading_ += dt_config_.dt *
                              (kf_.X_hat(3, 0) - kf_.X_hat(1, 0)) /
                              (dt_config_.robot_radius * 2.0);

    // gyro_heading = (real_right - real_left) / width
    // wheel_heading = (wheel_right - wheel_left) / width
    // gyro_heading + offset = wheel_heading
    // gyro_goal + offset = wheel_goal
    // offset = wheel_heading - gyro_heading

    // gyro_goal + wheel_heading - gyro_heading = wheel_goal
  }

  dt_openloop_.SetPosition(position, left_gear_, right_gear_);

  bool control_loop_driving = false;
  if (goal) {
    control_loop_driving = goal->control_loop_driving;

    dt_closedloop_.SetGoal(*goal);
    dt_openloop_.SetGoal(*goal);
  }

  dt_openloop_.Update();

  if (control_loop_driving) {
    dt_closedloop_.Update(output != NULL);
    dt_closedloop_.SetOutput(output);
  } else {
    dt_openloop_.SetOutput(output);
    // TODO(austin): Set profile to current spot.
    dt_closedloop_.Update(false);
  }

  // The output should now contain the shift request.

  // set the output status of the control loop state
  if (status) {
    status->robot_speed = (kf_.X_hat(1, 0) + kf_.X_hat(3, 0)) / 2.0;

    Eigen::Matrix<double, 2, 1> linear =
        dt_config_.LeftRightToLinear(kf_.X_hat());
    Eigen::Matrix<double, 2, 1> angular =
        dt_config_.LeftRightToAngular(kf_.X_hat());

    angular(0, 0) = integrated_kf_heading_;

    Eigen::Matrix<double, 4, 1> gyro_left_right =
        dt_config_.AngularLinearToLeftRight(linear, angular);

    status->estimated_left_position = gyro_left_right(0, 0);
    status->estimated_right_position = gyro_left_right(2, 0);

    status->estimated_left_velocity = gyro_left_right(1, 0);
    status->estimated_right_velocity = gyro_left_right(3, 0);
    status->output_was_capped = dt_closedloop_.output_was_capped();
    status->uncapped_left_voltage = kf_.U_uncapped(0, 0);
    status->uncapped_right_voltage = kf_.U_uncapped(1, 0);

    status->left_voltage_error = kf_.X_hat(4, 0);
    status->right_voltage_error = kf_.X_hat(5, 0);
    status->estimated_angular_velocity_error = kf_.X_hat(6, 0);
    status->estimated_heading = integrated_kf_heading_;
    status->ground_angle = down_estimator_.X_hat(0, 0) + dt_config_.down_offset;

    dt_openloop_.PopulateStatus(status);
    dt_closedloop_.PopulateStatus(status);
  }

  double left_voltage = 0.0;
  double right_voltage = 0.0;
  if (output) {
    left_voltage = output->left_voltage;
    right_voltage = output->right_voltage;
    left_high_requested_ = output->left_high;
    right_high_requested_ = output->right_high;
  }

  const double scalar = ::aos::robot_state->voltage_battery / 12.0;

  left_voltage *= scalar;
  right_voltage *= scalar;

  // To validate, look at the following:

  // Observed - dx/dt velocity for left, right.

  // Angular velocity error compared to the gyro
  // Gyro heading vs left-right
  // Voltage error.

  Eigen::Matrix<double, 2, 1> U;
  U(0, 0) = last_left_voltage_;
  U(1, 0) = last_right_voltage_;
  last_left_voltage_ = left_voltage;
  last_right_voltage_ = right_voltage;

  kf_.UpdateObserver(U);
}

void DrivetrainLoop::Zero(
    ::frc971::control_loops::DrivetrainQueue::Output *output) {
  output->left_voltage = 0;
  output->right_voltage = 0;
  output->left_high = dt_config_.default_high_gear;
  output->right_high = dt_config_.default_high_gear;
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
