#include "frc971/control_loops/drivetrain/drivetrain.h"

#include <sched.h>

#include <cmath>
#include <cstdio>
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
#include "frc971/wpilib/imu_batch_generated.h"

using ::aos::monotonic_clock;
namespace chrono = ::std::chrono;

namespace frc971 {
namespace control_loops {
namespace drivetrain {

namespace {
// Maximum variation to allow in the gyro when zeroing.
constexpr double kMaxYawGyroZeroingRange = 0.08;
}  // namespace

DrivetrainFilters::DrivetrainFilters(const DrivetrainConfig<double> &dt_config,
                                     ::aos::EventLoop *event_loop,
                                     LocalizerInterface *localizer)
    : dt_config_(dt_config),
      localizer_control_fetcher_(
          event_loop->MakeFetcher<LocalizerControl>("/drivetrain")),
      imu_values_fetcher_(
          event_loop->TryMakeFetcher<::frc971::IMUValuesBatch>("/drivetrain")),
      gyro_reading_fetcher_(
          event_loop->MakeFetcher<::frc971::sensors::GyroReading>(
              "/drivetrain")),
      down_estimator_(dt_config_),
      localizer_(localizer),
      kf_(dt_config_.make_kf_drivetrain_loop()),
      left_gear_(dt_config_.default_high_gear ? Gear::HIGH : Gear::LOW),
      right_gear_(dt_config_.default_high_gear ? Gear::HIGH : Gear::LOW),
      left_high_requested_(dt_config_.default_high_gear),
      right_high_requested_(dt_config_.default_high_gear) {
  last_voltage_.setZero();
  last_last_voltage_.setZero();
  frc971::controls::HPolytope<0>::Init();
  event_loop->OnRun([this]() {
    // On the first fetch, make sure that we are caught all the way up to the
    // present.
    if (imu_values_fetcher_.valid()) {
      imu_values_fetcher_.Fetch();
    }
  });
  if (dt_config.is_simulated) {
    down_estimator_.assume_perfect_gravity();
  }
}

flatbuffers::Offset<LocalizerState> DrivetrainFilters::PopulateLocalizerState(
    flatbuffers::FlatBufferBuilder *fbb) {
  return localizer_->PopulateStatus(fbb);
}
flatbuffers::Offset<ImuZeroerState> DrivetrainFilters::PopulateImuZeroerState(
    flatbuffers::FlatBufferBuilder *fbb) {
  return imu_zeroer_.PopulateStatus(fbb);
}

flatbuffers::Offset<DownEstimatorState>
DrivetrainFilters::PopulateDownEstimatorState(
    flatbuffers::FlatBufferBuilder *fbb,
    aos::monotonic_clock::time_point monotonic_now) {
  return down_estimator_.PopulateStatus(fbb, monotonic_now);
}

void DrivetrainFilters::Reset(aos::monotonic_clock::time_point monotonic_now,
                              const drivetrain::Position *position) {
  // If all the sensors got reset (e.g., due to wpilib_interface restarting),
  // reset the localizer and down estimator to avoid weird jumps in the
  // filters.
  down_estimator_.Reset();
  // Just reset the localizer to the current state, except for the encoders.
  LocalizerInterface::Ekf::State X_hat = localizer_->Xhat();
  X_hat(LocalizerInterface::StateIdx::kLeftEncoder) = position->left_encoder();
  X_hat(LocalizerInterface::StateIdx::kRightEncoder) =
      position->right_encoder();
  localizer_->Reset(monotonic_now, X_hat);
}

void DrivetrainFilters::Correct(aos::monotonic_clock::time_point monotonic_now,
                                const drivetrain::Position *position) {
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

  while (imu_values_fetcher_.valid() && imu_values_fetcher_.FetchNext()) {
    CHECK(imu_values_fetcher_->has_readings());
    last_gyro_time_ = monotonic_now;
    for (const IMUValues *value : *imu_values_fetcher_->readings()) {
      imu_zeroer_.InsertMeasurement(*value);
      if (!imu_zeroer_.Zeroed()) {
        continue;
      }
      const aos::monotonic_clock::time_point reading_time(
          std::chrono::nanoseconds(value->monotonic_timestamp_ns()));
      if (last_imu_update_ == aos::monotonic_clock::min_time) {
        last_imu_update_ = reading_time;
      }
      down_estimator_.Predict(imu_zeroer_.ZeroedGyro().value(),
                              imu_zeroer_.ZeroedAccel().value(),
                              reading_time - last_imu_update_);
      last_imu_update_ = reading_time;
    }
  }

  bool got_imu_reading = false;
  if (imu_values_fetcher_.valid() && imu_values_fetcher_.get() != nullptr) {
    imu_zeroer_.ProcessMeasurements();
    got_imu_reading = true;
    CHECK(imu_values_fetcher_->has_readings());
    if (imu_values_fetcher_->readings()->size() > 0) {
      const IMUValues *value = imu_values_fetcher_->readings()->Get(
          imu_values_fetcher_->readings()->size() - 1);
      switch (dt_config_.imu_type) {
        case IMUType::IMU_X:
          last_accel_ = -value->accelerometer_x();
          break;
        case IMUType::IMU_FLIPPED_X:
          last_accel_ = value->accelerometer_x();
          break;
        case IMUType::IMU_Y:
          last_accel_ = -value->accelerometer_y();
          break;
        case IMUType::IMU_Z:
          last_accel_ = value->accelerometer_z();
          break;
      }
    }
  }

  // TODO(austin): Signal the current gear to both loops.
  bool imu_zeroer_zeroed = imu_zeroer_.Zeroed();

  switch (dt_config_.gyro_type) {
    case GyroType::IMU_X_GYRO:
      if (got_imu_reading) {
        last_gyro_rate_ =
            imu_zeroer_zeroed ? imu_zeroer_.ZeroedGyro().value().x() : 0.0;
      }
      break;
    case GyroType::IMU_Y_GYRO:
      if (got_imu_reading) {
        last_gyro_rate_ =
            imu_zeroer_zeroed ? imu_zeroer_.ZeroedGyro().value().y() : 0.0;
      }
      break;
    case GyroType::IMU_Z_GYRO:
      if (got_imu_reading) {
        last_gyro_rate_ =
            imu_zeroer_zeroed ? imu_zeroer_.ZeroedGyro().value().z() : 0.0;
      }
      break;
    case GyroType::FLIPPED_IMU_Z_GYRO:
      if (got_imu_reading) {
        last_gyro_rate_ =
            imu_zeroer_zeroed ? -imu_zeroer_.ZeroedGyro().value().z() : 0.0;
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

  switch (dt_config_.gyro_type) {
    case GyroType::SPARTAN_GYRO:
    case GyroType::FLIPPED_SPARTAN_GYRO:
      if (!yaw_gyro_zero_.has_value()) {
        yaw_gyro_zeroer_.AddData(last_gyro_rate_);
        if (yaw_gyro_zeroer_.full() &&
            yaw_gyro_zeroer_.GetRange() < kMaxYawGyroZeroingRange) {
          yaw_gyro_zero_ = yaw_gyro_zeroer_.GetAverage()(0);
          VLOG(1) << "Zeroed to " << *yaw_gyro_zero_ << " Range "
                  << yaw_gyro_zeroer_.GetRange();
        }
      }
      ready_ = yaw_gyro_zero_.has_value();
      if (ready_) {
        last_gyro_rate_ = last_gyro_rate_ - yaw_gyro_zero_.value();
      }
      break;
    case GyroType::IMU_X_GYRO:
    case GyroType::IMU_Y_GYRO:
    case GyroType::IMU_Z_GYRO:
    case GyroType::FLIPPED_IMU_Z_GYRO:
      ready_ = imu_zeroer_.Zeroed();
      break;
  }

  // TODO(james): How aggressively can we fault here? If we fault to
  // aggressively, we might have issues during startup if wpilib_interface takes
  // too long to start publishing IMU measurements.
  if (monotonic_now > last_gyro_time_ + chrono::milliseconds(20)) {
    last_gyro_rate_ = 0.0;
  }

  if (imu_values_fetcher_.valid()) {
    localizer_->Update(
        {last_last_voltage_(kLeftVoltage), last_last_voltage_(kRightVoltage)},
        monotonic_now, position->left_encoder(), position->right_encoder(),
        down_estimator_.avg_recent_yaw_rates(),
        down_estimator_.avg_recent_accel());
  } else {
    localizer_->Update(
        {last_last_voltage_(kLeftVoltage), last_last_voltage_(kRightVoltage)},
        monotonic_now, position->left_encoder(), position->right_encoder(),
        last_gyro_rate_, Eigen::Vector3d::Zero());
  }

  // If we get a new message setting the absolute position, then reset the
  // localizer.
  if (localizer_control_fetcher_.Fetch()) {
    VLOG(1) << "localizer_control "
            << aos::FlatbufferToJson(localizer_control_fetcher_.get());
    localizer_->ResetPosition(
        monotonic_now, localizer_control_fetcher_->x(),
        localizer_control_fetcher_->y(), localizer_control_fetcher_->theta(),
        localizer_control_fetcher_->theta_uncertainty(),
        !localizer_control_fetcher_->keep_current_theta());
  }

  kf_.set_index(ControllerIndexFromGears());

  {
    Eigen::Matrix<double, 4, 1> Y;
    Y << position->left_encoder(), position->right_encoder(), last_gyro_rate_,
        last_accel_;
    kf_.Correct(Y);
  }
}

Eigen::Matrix<double, 2, 1> DrivetrainFilters::VoltageError() const {
  static_assert(kLeftError + 1 == kRightError);
  Eigen::Matrix<double, 2, 2> error_K;
  error_K << kf_.controller().K(kLeftVoltage, kLeftError), 0.0, 0.0,
      kf_.controller().K(kRightVoltage, kRightError);
  const Eigen::Matrix<double, 2, 1> voltage_error =
      error_K * kf_.X_hat().block<2, 1>(kLeftError, 0);
  return voltage_error;
}

void DrivetrainFilters::UpdateObserver(Eigen::Matrix<double, 2, 1> U) {
  last_last_voltage_ = last_voltage_;

  kf_.UpdateObserver(last_voltage_, dt_config_.dt);

  last_voltage_ = U;
}

int DrivetrainFilters::ControllerIndexFromGears() const {
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
flatbuffers::Offset<GearLogging> DrivetrainFilters::CreateGearLogging(
    flatbuffers::FlatBufferBuilder *fbb) const {
  GearLogging::Builder gear_logging_builder(*fbb);
  gear_logging_builder.add_left_state(static_cast<uint32_t>(left_gear_));
  gear_logging_builder.add_right_state(static_cast<uint32_t>(right_gear_));
  gear_logging_builder.add_left_loop_high(MaybeHigh(left_gear_));
  gear_logging_builder.add_right_loop_high(MaybeHigh(right_gear_));
  gear_logging_builder.add_controller_index(ControllerIndexFromGears());
  return gear_logging_builder.Finish();
}

Gear DrivetrainFilters::ComputeGear(
    double shifter_position, const constants::ShifterHallEffect &shifter_config,
    bool high_requested) const {
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

DrivetrainLoop::DrivetrainLoop(const DrivetrainConfig<double> &dt_config,
                               ::aos::EventLoop *event_loop,
                               LocalizerInterface *localizer,
                               const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, Status, Output>(event_loop,
                                                                    name),
      dt_config_(dt_config),
      filters_(dt_config, event_loop, localizer),
      dt_openloop_(dt_config_, filters_.kf()),
      dt_closedloop_(dt_config_, filters_.kf(), localizer),
      dt_spline_(dt_config_),
      dt_line_follow_(dt_config_, localizer->target_selector()) {
  event_loop->SetRuntimeRealtimePriority(30);
  for (size_t ii = 0; ii < trajectory_fetchers_.size(); ++ii) {
    trajectory_fetchers_[ii].fetcher =
        event_loop->MakeFetcher<fb::Trajectory>("/drivetrain");
  }
}

void DrivetrainLoop::UpdateTrajectoryFetchers() {
  if (dt_spline_.trajectory_count() >= trajectory_fetchers_.size()) {
    aos::monotonic_clock::time_point min_time = aos::monotonic_clock::max_time;
    size_t min_fetcher_index = 0;
    size_t fetcher_index = 0;
    // Find the oldest spline to forget.
    for (auto &fetcher : trajectory_fetchers_) {
      CHECK_NE(fetcher.fetcher.context().monotonic_event_time,
               monotonic_clock::min_time);
      if (fetcher.fetcher.context().monotonic_event_time < min_time &&
          !dt_spline_.IsCurrentTrajectory(fetcher.fetcher.get())) {
        min_time = fetcher.fetcher.context().monotonic_event_time;
        min_fetcher_index = fetcher_index;
      }
      ++fetcher_index;
    }

    dt_spline_.DeleteTrajectory(
        trajectory_fetchers_[min_fetcher_index].fetcher.get());
    trajectory_fetchers_[min_fetcher_index].in_use = false;
  }

  for (auto &fetcher : trajectory_fetchers_) {
    const fb::Trajectory *trajectory = fetcher.fetcher.get();
    // If the current fetcher is already being used by the SplineDrivetrain,
    // don't touch it.
    // We have to check both in_use and HasTrajectory because if
    // in_use is true and HasTrajectory() is false, that implies that the
    // SplineDrivetrain has finished executing the trajectory and disposed of
    // it; if in_use is false and HasTrajectory() is true, that implies that
    // this fetcher is at the same point in the queue as another fetcher, and
    // that the other fetcher is the one that we are using to keep the message
    // pinned.
    // TODO(james): Consider garbage-collecting splines once we run out of
    // fetchers.
    if (fetcher.in_use && dt_spline_.HasTrajectory(trajectory)) {
      continue;
    }
    fetcher.in_use = false;
    // Go through and find the next Trajectory that isn't already held by the
    // SplineDrivetrain, and add it.
    while (fetcher.fetcher.FetchNext()) {
      trajectory = fetcher.fetcher.get();
      if (!dt_spline_.HasTrajectory(trajectory)) {
        fetcher.in_use = true;
        dt_spline_.AddTrajectory(trajectory);
        break;
      }
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
  }

  if (WasReset()) {
    filters_.Reset(monotonic_now, position);
  }

  UpdateTrajectoryFetchers();

  filters_.Correct(monotonic_now, position);

  // Set the gear-logging parts of the status
  CHECK(status);
  flatbuffers::Offset<GearLogging> gear_logging_offset =
      filters_.CreateGearLogging(status->fbb());

  dt_openloop_.SetPosition(position, filters_.left_gear(),
                           filters_.right_gear());

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
      filters_.trajectory_state();

  {
    // TODO(james): The regular Kalman Filter's voltage error terms are
    // currently unusable--either don't use voltage error at all for the spline
    // following code, or use the EKF's voltage error estimates.
    const Eigen::Matrix<double, 2, 1> voltage_error =
        0 * filters_.VoltageError();
    dt_spline_.Update(
        output != nullptr && controller_type == ControllerType::SPLINE_FOLLOWER,
        trajectory_state, voltage_error);
  }

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
        dt_config_.LeftRightToLinear(filters_.DrivetrainXHat());
    Eigen::Matrix<double, 2, 1> angular =
        dt_config_.LeftRightToAngular(filters_.DrivetrainXHat());

    angular(0, 0) = filters_.localizer_theta();

    Eigen::Matrix<double, 4, 1> gyro_left_right =
        dt_config_.AngularLinearToLeftRight(linear, angular);

    const flatbuffers::Offset<CIMLogging> cim_logging_offset =
        dt_openloop_.PopulateShiftingStatus(status->fbb());

    const flatbuffers::Offset<PolyDriveLogging> poly_drive_logging_offset =
        dt_openloop_.PopulateStatus(status->fbb());

    const flatbuffers::Offset<DownEstimatorState> down_estimator_state_offset =
        filters_.PopulateDownEstimatorState(status->fbb(), monotonic_now);

    const flatbuffers::Offset<LocalizerState> localizer_offset =
        filters_.PopulateLocalizerState(status->fbb());

    const flatbuffers::Offset<ImuZeroerState> zeroer_offset =
        filters_.PopulateImuZeroerState(status->fbb());

    flatbuffers::Offset<LineFollowLogging> line_follow_logging_offset =
        dt_line_follow_.PopulateStatus(status);
    flatbuffers::Offset<TrajectoryLogging> trajectory_logging_offset =
        dt_spline_.MakeTrajectoryLogging(status);

    Status::Builder builder = status->MakeBuilder<Status>();

    dt_closedloop_.PopulateStatus(&builder);

    builder.add_estimated_left_position(gyro_left_right(kLeftPosition));
    builder.add_estimated_right_position(gyro_left_right(kRightPosition));

    builder.add_estimated_left_velocity(gyro_left_right(kLeftVelocity));
    builder.add_estimated_right_velocity(gyro_left_right(kRightVelocity));

    if (dt_spline_.enable()) {
      dt_spline_.PopulateStatus(&builder);
    } else {
      builder.add_robot_speed((filters_.DrivetrainXHat(kLeftVelocity) +
                               filters_.DrivetrainXHat(kRightVelocity)) /
                              2.0);
      builder.add_output_was_capped(dt_closedloop_.output_was_capped());
      builder.add_uncapped_left_voltage(
          filters_.DrivetrainUUncapped(kLeftVoltage));
      builder.add_uncapped_right_voltage(
          filters_.DrivetrainUUncapped(kRightVoltage));
    }

    builder.add_left_voltage_error(filters_.DrivetrainXHat(kLeftError));
    builder.add_right_voltage_error(filters_.DrivetrainXHat(kRightError));
    builder.add_estimated_angular_velocity_error(
        filters_.DrivetrainXHat(kAngularError));
    builder.add_estimated_heading(filters_.localizer_theta());

    builder.add_x(filters_.x());
    builder.add_y(filters_.y());
    builder.add_theta(::aos::math::NormalizeAngle(filters_.localizer_theta()));

    builder.add_cim_logging(cim_logging_offset);
    builder.add_poly_drive_logging(poly_drive_logging_offset);
    builder.add_gear_logging(gear_logging_offset);
    builder.add_line_follow_logging(line_follow_logging_offset);
    builder.add_trajectory_logging(trajectory_logging_offset);
    builder.add_down_estimator(down_estimator_state_offset);
    builder.add_localizer(localizer_offset);
    builder.add_zeroing(zeroer_offset);

    builder.add_send_failures(status_failure_counter_.failures());

    status_failure_counter_.Count(status->Send(builder.Finish()));
  }

  // If the filters aren't ready/valid, then disable all outputs (currently,
  // this only happens if the IMU is faulted or has not zeroed).
  // TODO(james): Add exceptions so that during competitive play the driver
  // can retain minimal control of the robot.
  if (!filters_.Ready()) {
    output_struct.left_voltage = 0.0;
    output_struct.right_voltage = 0.0;
  }

  double left_voltage = 0.0;
  double right_voltage = 0.0;
  if (output) {
    left_voltage = output_struct.left_voltage;
    right_voltage = output_struct.right_voltage;
    filters_.set_left_high_requested(output_struct.left_high);
    filters_.set_right_high_requested(output_struct.right_high);
  }

  const double scalar = robot_state().voltage_battery() / 12.0;

  left_voltage *= scalar;
  right_voltage *= scalar;

  // To validate, look at the following:

  // Observed - dx/dt velocity for left, right.

  // Angular velocity error compared to the gyro
  // Gyro heading vs left-right
  // Voltage error.

  {
    Eigen::Matrix<double, 2, 1> U;
    U(kLeftVoltage) = left_voltage;
    U(kRightVoltage) = right_voltage;
    filters_.UpdateObserver(U);
  }

  if (output) {
    output->CheckOk(output->Send(Output::Pack(*output->fbb(), &output_struct)));
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
