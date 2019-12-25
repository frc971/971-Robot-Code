#include "frc971/control_loops/drivetrain/drivetrain.h"

#include <stdio.h>
#include <sched.h>
#include <cmath>
#include <memory>
#include "Eigen/Dense"

#include "aos/logging/logging.h"
#include "frc971/control_loops/drivetrain/down_estimator.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/drivetrain/polydrivetrain.h"
#include "frc971/control_loops/drivetrain/ssdrivetrain.h"
#include "frc971/control_loops/runge_kutta.h"
#include "frc971/queues/gyro_generated.h"
#include "frc971/shifter_hall_effect.h"
#include "frc971/wpilib/imu_generated.h"

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;

namespace frc971 {
namespace control_loops {
namespace drivetrain {

DrivetrainLoop::DrivetrainLoop(const DrivetrainConfig<double> &dt_config,
                               ::aos::EventLoop *event_loop,
                               LocalizerInterface *localizer,
                               const ::std::string &name)
    : aos::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                 name),
      dt_config_(dt_config),
      localizer_control_fetcher_(
          event_loop->MakeFetcher<LocalizerControl>("/drivetrain")),
      imu_values_fetcher_(
          event_loop->MakeFetcher<::frc971::IMUValues>("/drivetrain")),
      gyro_reading_fetcher_(
          event_loop->MakeFetcher<::frc971::sensors::GyroReading>(
              "/drivetrain")),
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
  event_loop->SetRuntimeRealtimePriority(30);
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
    const drivetrain::Goal *goal, const drivetrain::Position *position,
    aos::Sender<drivetrain::Output>::Builder *output,
    aos::Sender<drivetrain::Status>::Builder *status) {
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
      left_gear_ = ComputeGear(position->left_shifter_position(),
                               dt_config_.left_drive, left_high_requested_);
      right_gear_ = ComputeGear(position->right_shifter_position(),
                                dt_config_.right_drive, right_high_requested_);
      break;
    case ShifterType::NO_SHIFTER:
      break;
  }

  kf_.set_index(ControllerIndexFromGears());

  flatbuffers::Offset<GearLogging> gear_logging_offset;
  // Set the gear-logging parts of the status
  if (status) {
    GearLogging::Builder gear_logging_builder =
        status->MakeBuilder<GearLogging>();
    gear_logging_builder.add_left_state(static_cast<uint32_t>(left_gear_));
    gear_logging_builder.add_right_state(static_cast<uint32_t>(right_gear_));
    gear_logging_builder.add_left_loop_high(MaybeHigh(left_gear_));
    gear_logging_builder.add_right_loop_high(MaybeHigh(right_gear_));
    gear_logging_builder.add_controller_index(kf_.index());
    gear_logging_offset = gear_logging_builder.Finish();
  }

  const bool is_latest_imu_values = imu_values_fetcher_.Fetch();
  if (is_latest_imu_values) {
    const double rate = -imu_values_fetcher_->gyro_y();
    const double accel_squared =
        ::std::pow(imu_values_fetcher_->accelerometer_x(), 2.0) +
        ::std::pow(imu_values_fetcher_->accelerometer_y(), 2.0) +
        ::std::pow(imu_values_fetcher_->accelerometer_z(), 2.0);
    const double angle = ::std::atan2(imu_values_fetcher_->accelerometer_x(),
                                      imu_values_fetcher_->accelerometer_z()) +
                         0.008;

    switch (dt_config_.imu_type) {
      case IMUType::IMU_X:
        last_accel_ = -imu_values_fetcher_->accelerometer_x();
        break;
      case IMUType::IMU_FLIPPED_X:
        last_accel_ = imu_values_fetcher_->accelerometer_x();
        break;
      case IMUType::IMU_Y:
        last_accel_ = -imu_values_fetcher_->accelerometer_y();
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
        last_gyro_rate_ = imu_values_fetcher_->gyro_x();
        last_gyro_time_ = monotonic_now;
      }
      break;
    case GyroType::IMU_Y_GYRO:
      if (is_latest_imu_values) {
        last_gyro_rate_ = imu_values_fetcher_->gyro_y();
        last_gyro_time_ = monotonic_now;
      }
      break;
    case GyroType::IMU_Z_GYRO:
      if (is_latest_imu_values) {
        last_gyro_rate_ = imu_values_fetcher_->gyro_z();
        last_gyro_time_ = monotonic_now;
      }
      break;
    case GyroType::FLIPPED_IMU_Z_GYRO:
      if (is_latest_imu_values) {
        last_gyro_rate_ = -imu_values_fetcher_->gyro_z();
        last_gyro_time_ = monotonic_now;
      }
      break;
    case GyroType::SPARTAN_GYRO:
      if (gyro_reading_fetcher_.Fetch()) {
        last_gyro_rate_ = gyro_reading_fetcher_->velocity();
        last_gyro_time_ = monotonic_now;
      }
      break;
    case GyroType::FLIPPED_SPARTAN_GYRO:
      if (gyro_reading_fetcher_.Fetch()) {
        last_gyro_rate_ = -gyro_reading_fetcher_->velocity();
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
    Y << position->left_encoder(), position->right_encoder(), last_gyro_rate_,
        last_accel_;
    kf_.Correct(Y);
    // If we get a new message setting the absolute position, then reset the
    // localizer.
    // TODO(james): Use a watcher (instead of a fetcher) once we support it in
    // simulation.
    if (localizer_control_fetcher_.Fetch()) {
      VLOG(1) << "localizer_control "
              << aos::FlatbufferToJson(localizer_control_fetcher_.get());
      localizer_->ResetPosition(
          monotonic_now, localizer_control_fetcher_->x(),
          localizer_control_fetcher_->y(), localizer_control_fetcher_->theta(),
          localizer_control_fetcher_->theta_uncertainty(),
          !localizer_control_fetcher_->keep_current_theta());
    }
    localizer_->Update({last_last_left_voltage_, last_last_right_voltage_},
                       monotonic_now, position->left_encoder(),
                       position->right_encoder(), last_gyro_rate_, last_accel_);
  }

  dt_openloop_.SetPosition(position, left_gear_, right_gear_);

  ControllerType controller_type = ControllerType::POLYDRIVE;
  if (goal) {
    controller_type = goal->controller_type();

    dt_closedloop_.SetGoal(goal);
    dt_openloop_.SetGoal(goal->wheel(), goal->throttle(), goal->quickturn(),
                         goal->highgear());
    dt_spline_.SetGoal(goal);
    dt_line_follow_.SetGoal(monotonic_now, goal);
  }

  dt_openloop_.Update(robot_state().voltage_battery());

  dt_closedloop_.Update(output != nullptr &&
                        controller_type == ControllerType::MOTION_PROFILE);

  const Eigen::Matrix<double, 5, 1> trajectory_state =
      (Eigen::Matrix<double, 5, 1>() << localizer_->x(), localizer_->y(),
       localizer_->theta(), localizer_->left_velocity(),
       localizer_->right_velocity())
          .finished();

  dt_spline_.Update(
      output != nullptr && controller_type == ControllerType::SPLINE_FOLLOWER,
      trajectory_state);

  dt_line_follow_.Update(monotonic_now, trajectory_state);

  OutputT output_struct;

  switch (controller_type) {
    case ControllerType::POLYDRIVE:
      dt_openloop_.SetOutput(output != nullptr ? &output_struct : nullptr);
      break;
    case ControllerType::MOTION_PROFILE:
      dt_closedloop_.SetOutput(output != nullptr ? &output_struct : nullptr);
      break;
    case ControllerType::SPLINE_FOLLOWER:
      dt_spline_.SetOutput(output != nullptr ? &output_struct : nullptr);
      break;
    case ControllerType::LINE_FOLLOWER:
      if (!dt_line_follow_.SetOutput(output != nullptr ? &output_struct
                                                       : nullptr)) {
        // If the line follow drivetrain was unable to execute (generally due to
        // not having a target), execute the regular teleop drivetrain.
        dt_openloop_.SetOutput(output != nullptr ? &output_struct : nullptr);
      }
      break;
  }

  // The output should now contain the shift request.

  // set the output status of the control loop state
  if (status) {
    Eigen::Matrix<double, 2, 1> linear =
        dt_config_.LeftRightToLinear(kf_.X_hat());
    Eigen::Matrix<double, 2, 1> angular =
        dt_config_.LeftRightToAngular(kf_.X_hat());

    angular(0, 0) = localizer_->theta();

    Eigen::Matrix<double, 4, 1> gyro_left_right =
        dt_config_.AngularLinearToLeftRight(linear, angular);

    const flatbuffers::Offset<CIMLogging> cim_logging_offset =
        dt_openloop_.PopulateStatus(status->fbb());

    flatbuffers::Offset<LineFollowLogging> line_follow_logging_offset =
        dt_line_follow_.PopulateStatus(status);
    flatbuffers::Offset<TrajectoryLogging> trajectory_logging_offset =
        dt_spline_.MakeTrajectoryLogging(status);

    StatusBuilder builder = status->MakeBuilder<Status>();

    dt_closedloop_.PopulateStatus(&builder);

    builder.add_estimated_left_position(gyro_left_right(0, 0));
    builder.add_estimated_right_position(gyro_left_right(2, 0));

    builder.add_estimated_left_velocity(gyro_left_right(1, 0));
    builder.add_estimated_right_velocity(gyro_left_right(3, 0));

    if (dt_spline_.enable()) {
      dt_spline_.PopulateStatus(&builder);
    } else {
      builder.add_robot_speed((kf_.X_hat(1) + kf_.X_hat(3)) / 2.0);
      builder.add_output_was_capped(dt_closedloop_.output_was_capped());
      builder.add_uncapped_left_voltage(kf_.U_uncapped(0, 0));
      builder.add_uncapped_right_voltage(kf_.U_uncapped(1, 0));
    }

    builder.add_left_voltage_error(kf_.X_hat(4));
    builder.add_right_voltage_error(kf_.X_hat(5));
    builder.add_estimated_angular_velocity_error(kf_.X_hat(6));
    builder.add_estimated_heading(localizer_->theta());

    builder.add_x(localizer_->x());
    builder.add_y(localizer_->y());
    builder.add_theta(::aos::math::NormalizeAngle(localizer_->theta()));

    builder.add_ground_angle(down_estimator_.X_hat(0) + dt_config_.down_offset);
    builder.add_cim_logging(cim_logging_offset);
    builder.add_gear_logging(gear_logging_offset);
    builder.add_line_follow_logging(line_follow_logging_offset);
    builder.add_trajectory_logging(trajectory_logging_offset);
    status->Send(builder.Finish());
  }

  double left_voltage = 0.0;
  double right_voltage = 0.0;
  if (output) {
    left_voltage = output_struct.left_voltage;
    right_voltage = output_struct.right_voltage;
    left_high_requested_ = output_struct.left_high;
    right_high_requested_ = output_struct.right_high;
  }

  const double scalar = robot_state().voltage_battery() / 12.0;

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

  if (output) {
    output->Send(Output::Pack(*output->fbb(), &output_struct));
  }
}

flatbuffers::Offset<Output> DrivetrainLoop::Zero(
    aos::Sender<Output>::Builder *output) {
  Output::Builder builder = output->MakeBuilder<Output>();
  builder.add_left_voltage(0);
  builder.add_right_voltage(0);
  builder.add_left_high(dt_config_.default_high_gear);
  builder.add_right_high(dt_config_.default_high_gear);
  return builder.Finish();
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
