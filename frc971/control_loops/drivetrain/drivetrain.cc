#include "frc971/control_loops/drivetrain/drivetrain.h"

#include <stdio.h>
#include <sched.h>
#include <cmath>
#include <memory>
#include "Eigen/Dense"

#include "aos/logging/logging.h"
#include "aos/logging/queue_logging.h"
#include "aos/logging/matrix_logging.h"

#include "frc971/control_loops/drivetrain/down_estimator.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/polydrivetrain.h"
#include "frc971/control_loops/drivetrain/ssdrivetrain.h"
#include "frc971/control_loops/runge_kutta.h"
#include "frc971/queues/gyro.q.h"
#include "frc971/shifter_hall_effect.h"
#include "frc971/wpilib/imu.q.h"

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;

namespace frc971 {
namespace control_loops {
namespace drivetrain {

DrivetrainLoop::DrivetrainLoop(const DrivetrainConfig<double> &dt_config,
                               ::aos::EventLoop *event_loop,
                               LocalizerInterface *localizer,
                               const ::std::string &name)
    : aos::controls::ControlLoop<::frc971::control_loops::DrivetrainQueue>(
          event_loop, name),
      dt_config_(dt_config),
      localizer_control_fetcher_(event_loop->MakeFetcher<LocalizerControl>(
          ".frc971.control_loops.drivetrain.localizer_control")),
      imu_values_fetcher_(
          event_loop->MakeFetcher<::frc971::IMUValues>(".frc971.imu_values")),
      gyro_reading_fetcher_(
          event_loop->MakeFetcher<::frc971::sensors::GyroReading>(
              ".frc971.sensors.gyro_reading")),
      localizer_(localizer),
      kf_(dt_config_.make_kf_drivetrain_loop()),
      dt_openloop_(dt_config_, &kf_),
      dt_closedloop_(dt_config_, &kf_, localizer_),
      dt_spline_(dt_config_),
      dt_line_follow_(dt_config_, localizer->target_selector()),
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
  const monotonic_clock::time_point monotonic_now =
      event_loop()->monotonic_now();

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

  kf_.set_index(ControllerIndexFromGears());

  // Set the gear-logging parts of the status
  if (status) {
    status->gear_logging.left_state = static_cast<uint32_t>(left_gear_);
    status->gear_logging.right_state = static_cast<uint32_t>(right_gear_);
    status->gear_logging.left_loop_high = MaybeHigh(left_gear_);
    status->gear_logging.right_loop_high = MaybeHigh(right_gear_);
    status->gear_logging.controller_index = kf_.index();
  }

  const bool is_latest_imu_values = imu_values_fetcher_.Fetch();
  if (is_latest_imu_values) {
    const double rate = -imu_values_fetcher_->gyro_y;
    const double accel_squared =
        ::std::pow(imu_values_fetcher_->accelerometer_x, 2.0) +
        ::std::pow(imu_values_fetcher_->accelerometer_y, 2.0) +
        ::std::pow(imu_values_fetcher_->accelerometer_z, 2.0);
    const double angle = ::std::atan2(imu_values_fetcher_->accelerometer_x,
                                      imu_values_fetcher_->accelerometer_z) +
                         0.008;

    switch (dt_config_.imu_type) {
      case IMUType::IMU_X:
        last_accel_ = -imu_values_fetcher_->accelerometer_x;
        break;
      case IMUType::IMU_FLIPPED_X:
        last_accel_ = imu_values_fetcher_->accelerometer_x;
        break;
      case IMUType::IMU_Y:
        last_accel_ = -imu_values_fetcher_->accelerometer_y;
        break;
    }

    if (accel_squared > 1.03 || accel_squared < 0.97) {
      AOS_LOG(DEBUG, "New IMU value, rejecting reading\n");
    } else {
      // -y is our gyro.
      // z accel is down
      // x accel is the front of the robot pointed down.
      Eigen::Matrix<double, 1, 1> Y;
      Y(0, 0) = angle;
      down_estimator_.Correct(Y);
    }

    AOS_LOG(DEBUG,
            "New IMU value, rate is %f, angle %f, fused %f, bias "
            "%f\n",
            rate, angle, down_estimator_.X_hat(0), down_estimator_.X_hat(1));
    down_U_(0, 0) = rate;
  }
  down_estimator_.UpdateObserver(down_U_, ::aos::controls::kLoopFrequency);

  // TODO(austin): Signal the current gear to both loops.

  switch (dt_config_.gyro_type) {
    case GyroType::IMU_X_GYRO:
      if (is_latest_imu_values) {
        AOS_LOG_STRUCT(DEBUG, "using", *imu_values_fetcher_.get());
        last_gyro_rate_ = imu_values_fetcher_->gyro_x;
        last_gyro_time_ = monotonic_now;
      }
      break;
    case GyroType::IMU_Y_GYRO:
      if (is_latest_imu_values) {
        AOS_LOG_STRUCT(DEBUG, "using", *imu_values_fetcher_.get());
        last_gyro_rate_ = imu_values_fetcher_->gyro_y;
        last_gyro_time_ = monotonic_now;
      }
      break;
    case GyroType::IMU_Z_GYRO:
      if (is_latest_imu_values) {
        AOS_LOG_STRUCT(DEBUG, "using", *imu_values_fetcher_.get());
        last_gyro_rate_ = imu_values_fetcher_->gyro_z;
        last_gyro_time_ = monotonic_now;
      }
      break;
    case GyroType::FLIPPED_IMU_Z_GYRO:
      if (is_latest_imu_values) {
        AOS_LOG_STRUCT(DEBUG, "using", *imu_values_fetcher_.get());
        last_gyro_rate_ = -imu_values_fetcher_->gyro_z;
        last_gyro_time_ = monotonic_now;
      }
      break;
    case GyroType::SPARTAN_GYRO:
      if (gyro_reading_fetcher_.Fetch()) {
        AOS_LOG_STRUCT(DEBUG, "using", *gyro_reading_fetcher_.get());
        last_gyro_rate_ = gyro_reading_fetcher_->velocity;
        last_gyro_time_ = monotonic_now;
      }
      break;
    case GyroType::FLIPPED_SPARTAN_GYRO:
      if (gyro_reading_fetcher_.Fetch()) {
        AOS_LOG_STRUCT(DEBUG, "using", *gyro_reading_fetcher_.get());
        last_gyro_rate_ = -gyro_reading_fetcher_->velocity;
        last_gyro_time_ = monotonic_now;
      }
      break;
    default:
      AOS_LOG(FATAL, "invalid gyro configured");
      break;
  }

  if (monotonic_now > last_gyro_time_ + chrono::milliseconds(20)) {
    last_gyro_rate_ = 0.0;
  }

  {
    Eigen::Matrix<double, 4, 1> Y;
    Y << position->left_encoder, position->right_encoder, last_gyro_rate_,
        last_accel_;
    kf_.Correct(Y);
    // If we get a new message setting the absolute position, then reset the
    // localizer.
    // TODO(james): Use a watcher (instead of a fetcher) once we support it in
    // simulation.
    if (localizer_control_fetcher_.Fetch()) {
      AOS_LOG_STRUCT(DEBUG, "localizer_control", *localizer_control_fetcher_);
      localizer_->ResetPosition(
          monotonic_now, localizer_control_fetcher_->x,
          localizer_control_fetcher_->y, localizer_control_fetcher_->theta,
          localizer_control_fetcher_->theta_uncertainty,
          !localizer_control_fetcher_->keep_current_theta);
    }
    localizer_->Update({last_last_left_voltage_, last_last_right_voltage_},
                       monotonic_now, position->left_encoder,
                       position->right_encoder, last_gyro_rate_, last_accel_);
  }

  dt_openloop_.SetPosition(position, left_gear_, right_gear_);

  int controller_type = 0;
  if (goal) {
    controller_type = goal->controller_type;

    dt_closedloop_.SetGoal(*goal);
    dt_openloop_.SetGoal(*goal);
    dt_spline_.SetGoal(*goal);
    dt_line_follow_.SetGoal(monotonic_now, *goal);
  }

  dt_openloop_.Update(robot_state().voltage_battery);

  dt_closedloop_.Update(output != NULL && controller_type == 1);

  const Eigen::Matrix<double, 5, 1> trajectory_state =
      (Eigen::Matrix<double, 5, 1>() << localizer_->x(), localizer_->y(),
       localizer_->theta(), localizer_->left_velocity(),
       localizer_->right_velocity())
          .finished();

  dt_spline_.Update(output != NULL && controller_type == 2, trajectory_state);

  dt_line_follow_.Update(monotonic_now, trajectory_state);

  switch (controller_type) {
    case 0:
      dt_openloop_.SetOutput(output);
      break;
    case 1:
      dt_closedloop_.SetOutput(output);
      break;
    case 2:
      dt_spline_.SetOutput(output);
      break;
    case 3:
      if (!dt_line_follow_.SetOutput(output)) {
        // If the line follow drivetrain was unable to execute (generally due to
        // not having a target), execute the regular teleop drivetrain.
        dt_openloop_.SetOutput(output);
      }
      break;
  }

  // The output should now contain the shift request.

  // set the output status of the control loop state
  if (status) {
    status->robot_speed = (kf_.X_hat(1) + kf_.X_hat(3)) / 2.0;

    Eigen::Matrix<double, 2, 1> linear =
        dt_config_.LeftRightToLinear(kf_.X_hat());
    Eigen::Matrix<double, 2, 1> angular =
        dt_config_.LeftRightToAngular(kf_.X_hat());

    angular(0, 0) = localizer_->theta();

    Eigen::Matrix<double, 4, 1> gyro_left_right =
        dt_config_.AngularLinearToLeftRight(linear, angular);

    status->estimated_left_position = gyro_left_right(0, 0);
    status->estimated_right_position = gyro_left_right(2, 0);

    status->estimated_left_velocity = gyro_left_right(1, 0);
    status->estimated_right_velocity = gyro_left_right(3, 0);
    status->output_was_capped = dt_closedloop_.output_was_capped();
    status->uncapped_left_voltage = kf_.U_uncapped(0, 0);
    status->uncapped_right_voltage = kf_.U_uncapped(1, 0);

    status->left_voltage_error = kf_.X_hat(4);
    status->right_voltage_error = kf_.X_hat(5);
    status->estimated_angular_velocity_error = kf_.X_hat(6);
    status->estimated_heading = localizer_->theta();

    status->x = localizer_->x();
    status->y = localizer_->y();
    status->theta = ::aos::math::NormalizeAngle(localizer_->theta());

    status->ground_angle = down_estimator_.X_hat(0) + dt_config_.down_offset;

    dt_openloop_.PopulateStatus(status);
    dt_closedloop_.PopulateStatus(status);
    dt_spline_.PopulateStatus(status);
    dt_line_follow_.PopulateStatus(status);
  }

  double left_voltage = 0.0;
  double right_voltage = 0.0;
  if (output) {
    left_voltage = output->left_voltage;
    right_voltage = output->right_voltage;
    left_high_requested_ = output->left_high;
    right_high_requested_ = output->right_high;
  }

  const double scalar = robot_state().voltage_battery / 12.0;

  left_voltage *= scalar;
  right_voltage *= scalar;

  // To validate, look at the following:

  // Observed - dx/dt velocity for left, right.

  // Angular velocity error compared to the gyro
  // Gyro heading vs left-right
  // Voltage error.

  last_last_left_voltage_ = last_left_voltage_;
  last_last_right_voltage_ = last_right_voltage_;
  Eigen::Matrix<double, 2, 1> U;
  U(0, 0) = last_left_voltage_;
  U(1, 0) = last_right_voltage_;
  last_left_voltage_ = left_voltage;
  last_right_voltage_ = right_voltage;

  last_state_ = kf_.X_hat();
  kf_.UpdateObserver(U, dt_config_.dt);
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
