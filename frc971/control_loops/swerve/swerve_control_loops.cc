#include "frc971/control_loops/swerve/swerve_control_loops.h"

#include "aos/commonmath.h"

ABSL_FLAG(int, swerve_priority, 20, "");
ABSL_FLAG(int, swerve_skip_iters, 1,
          "Set to the number of iterations to skip + 1.");
ABSL_FLAG(bool, drive_heading, true, "");

namespace frc971::control_loops::swerve {

SwerveControlLoops::SwerveControlLoops(
    ::aos::EventLoop *event_loop,
    const frc971::control_loops::
        StaticZeroingSingleDOFProfiledSubsystemCommonParams *,
    const SwerveZeroing *zeroing_params,
    const NaiveEstimator::Parameters &params,
    const LinearVelocityController::ControllerWeights &lvc_weights,
    const ::std::string &name)
    : frc971::controls::ControlLoop<Goal, Position, StatusStatic, OutputStatic>(
          event_loop, name),
      can_position_fetcher_(event_loop->MakeFetcher<CanPosition>(name)),
      gyro_fetcher_(event_loop->TryMakeFetcher<::frc971::sensors::GyroReading>(
          "/drivetrain")),
      auto_goal_fetcher_(event_loop->MakeFetcher<Goal>("/autonomous")),
      auto_align_goal_fetcher_(
          event_loop->MakeFetcher<Goal>("/autonomous_auto_align")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      imu_fetcher_(
          event_loop->TryMakeFetcher<::frc971::IMUValuesBatch>("/localizer")),
      naive_estimator_(event_loop, zeroing_params, params),
      auto_align_(event_loop),
      velocity_controller_(
          LinearVelocityController::MakeParameters(lvc_weights, params),
          params),
      inverse_kinematics_(params),
      velocity_ekf_(params) {
  if (absl::GetFlag(FLAGS_swerve_priority) > 0) {
    event_loop->SetRuntimeRealtimePriority(
        absl::GetFlag(FLAGS_swerve_priority));
  }
}

void SwerveControlLoops::RunIteration(
    const Goal *goal, const Position *position,
    aos::Sender<OutputStatic>::StaticBuilder *output_builder,
    aos::Sender<StatusStatic>::StaticBuilder *status_builder) {
  ++iteration_counter_;
  if (iteration_counter_ % absl::GetFlag(FLAGS_swerve_skip_iters) != 0) {
    return;
  }
  const aos::monotonic_clock::time_point profiling_start_time =
      aos::monotonic_clock::now();
  const aos::monotonic_clock::time_point now =
      event_loop()->context().monotonic_event_time;

  can_position_fetcher_.Fetch();

  if (gyro_fetcher_.valid()) {
    gyro_fetcher_.Fetch();
  }
  if (imu_fetcher_.valid()) {
    while (imu_fetcher_.FetchNext()) {
      for (const IMUValues *values : *imu_fetcher_->readings()) {
        imu_zeroer_.InsertMeasurement(*values);
      }
    }
    imu_zeroer_.ProcessMeasurements();
  }

  std::optional<NaiveEstimator::State> current_state;
  std::optional<double> gyro_rate;
  if (gyro_fetcher_.valid() && gyro_fetcher_.get() != nullptr) {
    gyro_rate = gyro_fetcher_->velocity();
    if (!yaw_gyro_zero_.has_value()) {
      yaw_gyro_zeroer_.AddData(gyro_rate.value());
      // Maximum variation to allow in the gyro when zeroing.
      constexpr double kMaxYawGyroZeroingRange = 0.15;
      if (yaw_gyro_zeroer_.full() &&
          yaw_gyro_zeroer_.GetRange() < kMaxYawGyroZeroingRange) {
        yaw_gyro_zero_ = yaw_gyro_zeroer_.GetAverage()(0);
        VLOG(1) << "Zeroed to " << *yaw_gyro_zero_ << " Range "
                << yaw_gyro_zeroer_.GetRange();
      }
    }
    if (yaw_gyro_zero_.has_value()) {
      gyro_rate = gyro_rate.value() - yaw_gyro_zero_.value();
    } else {
      gyro_rate = 0.0;
    }
  }

  std::optional<Eigen::Vector3d> zeroed_gyro = imu_zeroer_.ZeroedGyro();
  std::optional<Eigen::Vector3d> zeroed_accel = imu_zeroer_.ZeroedAccel();

  if (zeroed_gyro.has_value()) {
    gyro_rate = zeroed_gyro.value().z();
  }

  if (gyro_rate.has_value() && zeroed_accel.has_value() &&
      can_position_fetcher_.get() != nullptr) {
    current_state = naive_estimator_.Update(
        now, position, can_position_fetcher_.get(), gyro_rate.value(),
        zeroed_accel->x(), zeroed_accel->y());
    if (!ekf_initialized_) {
      velocity_ekf_.Initialize(
          now, current_state.value().head(States::kNumVelocityStates));
      ekf_initialized_ = true;
    }
    velocity_ekf_.Update(
        now,
        Eigen::Matrix<Scalar, 4, 1>{{current_state.value()(States::kThetas0)},
                                    {current_state.value()(States::kThetas1)},
                                    {current_state.value()(States::kThetas2)},
                                    {current_state.value()(States::kThetas3)}},
        {
            aos::monotonic_clock::time_point{
                std::chrono::nanoseconds(can_position_fetcher_->front_left()
                                             ->translation()
                                             ->timestamp())},
            aos::monotonic_clock::time_point{
                std::chrono::nanoseconds(can_position_fetcher_->front_right()
                                             ->translation()
                                             ->timestamp())},
            aos::monotonic_clock::time_point{
                std::chrono::nanoseconds(can_position_fetcher_->back_left()
                                             ->translation()
                                             ->timestamp())},
            aos::monotonic_clock::time_point{
                std::chrono::nanoseconds(can_position_fetcher_->back_right()
                                             ->translation()
                                             ->timestamp())},
        },
        Eigen::Matrix<Scalar, 4, 1>{
            {can_position_fetcher_->front_left()->translation()->position()},
            {can_position_fetcher_->front_right()->translation()->position()},
            {can_position_fetcher_->back_left()->translation()->position()},
            {can_position_fetcher_->back_right()->translation()->position()}},
        gyro_rate.value(), U_,
        (status_builder != nullptr) ? status_builder->get()->add_velocity_ekf()
                                    : nullptr);
  }

  const aos::monotonic_clock::time_point estimation_done =
      aos::monotonic_clock::now();
  U_.setZero();
  std::optional<LinearVelocityController::ControllerResult> controller_result;
  joystick_state_fetcher_.Fetch();
  auto_goal_fetcher_.Fetch();
  if (joystick_state_fetcher_.get() != nullptr &&
      joystick_state_fetcher_->autonomous() &&
      auto_goal_fetcher_.get() != nullptr) {
    goal = auto_goal_fetcher_.get();
  }
  if (goal != nullptr && current_state.has_value()) {
    CHECK_NE(goal->has_linear_velocity_goal(), goal->has_joystick_goal());
    if (goal->has_linear_velocity_goal()) {
      NaiveEstimator::State goal_state =
          ToEigenOrDie<NaiveEstimator::States::kNumVelocityStates, 1>(
              *goal->linear_velocity_goal()->state())
              .cast<Scalar>();
      controller_result = velocity_controller_.RunRawController(
          current_state.value().head(States::kNumVelocityStates),
          goal_state.head(States::kNumVelocityStates),
          ToEigenOrDie<8, 1>(*goal->linear_velocity_goal()->input())
              .cast<Scalar>());
    } else if (goal->has_joystick_goal()) {
      if (!desired_heading_.has_value()) {
        desired_heading_ = JoystickHeadingGoal{
            .heading = current_state.value()(NaiveEstimator::States::kTheta),
            .last_time = now};
      }
      NaiveEstimator::State kinematics_state = current_state.value();

      if (goal->joystick_goal()->foc_override()) {
        naive_estimator_.use_localizer_theta(true);
      } else {
        naive_estimator_.use_localizer_theta(false);
      }

      Scalar goal_omega = 0;

      auto_align_goal_fetcher_.Fetch();
      if (goal->joystick_goal()->auto_align()) {
        auto_align_.Iterate();
        auto_align_goal_fetcher_.Fetch();
        if (!auto_align_goal_fetcher_->joystick_goal()->has_vx()) {
          kinematics_state(NaiveEstimator::States::kVx) =
              goal->joystick_goal()->vx();
          kinematics_state(NaiveEstimator::States::kVy) =
              goal->joystick_goal()->vy();
        } else {
          kinematics_state(NaiveEstimator::States::kVx) =
              auto_align_goal_fetcher_.get()->joystick_goal()->vx();
          kinematics_state(NaiveEstimator::States::kVy) =
              auto_align_goal_fetcher_.get()->joystick_goal()->vy();
        }
        goal_omega = auto_align_goal_fetcher_.get()->joystick_goal()->omega();
      } else {
        kinematics_state(NaiveEstimator::States::kVx) =
            goal->joystick_goal()->vx();
        kinematics_state(NaiveEstimator::States::kVy) =
            goal->joystick_goal()->vy();
        goal_omega = goal->joystick_goal()->omega();
      }

      const Scalar current_omega =
          kinematics_state(NaiveEstimator::States::kOmega);
      const Scalar current_theta =
          kinematics_state(NaiveEstimator::States::kTheta);
      kinematics_state(NaiveEstimator::States::kOmega) = goal_omega;
      const Scalar omega_capped =
          (goal_omega > 0)
              ? std::min(goal_omega, std::max<Scalar>(0.0, current_omega))
              : std::max(goal_omega, std::min<Scalar>(0.0, current_omega));
      desired_heading_.value().heading +=
          omega_capped * aos::time::DurationInSeconds(
                             now - desired_heading_.value().last_time);
      desired_heading_.value().heading =
          aos::Clip(desired_heading_.value().heading, current_theta - 0.1,
                    current_theta + 0.1);
      desired_heading_.value().last_time = now;
      if (absl::GetFlag(FLAGS_drive_heading)) {
        kinematics_state(NaiveEstimator::States::kTheta) =
            desired_heading_.value().heading;
      }

      controller_result = velocity_controller_.RunRawController(
          current_state.value().head(States::kNumVelocityStates),
          inverse_kinematics_.Solve(
              kinematics_state.head(States::kNumVelocityStates)),
          Eigen::Matrix<Scalar, 8, 1>::Zero());
    } else {
      LOG(FATAL) << "Unreachable";
    }
    U_ = controller_result->U.cast<float>();
  }

  const aos::monotonic_clock::time_point controller_done =
      aos::monotonic_clock::now();

  constexpr Scalar kMaxDriveCurrent = 100.0;

  const Scalar reference_drive_current = std::max<Scalar>(
      kMaxDriveCurrent,
      std::max(
          {std::abs(U_(InputStates::kIs0)), std::abs(U_(InputStates::kIs1)),
           std::abs(U_(InputStates::kIs2)), std::abs(U_(InputStates::kIs3))}));
  const Scalar drive_current_scalar =
      kMaxDriveCurrent / reference_drive_current;

  if (output_builder != nullptr) {
    OutputStatic *output = output_builder->get();

    {
      auto module_output = output->add_front_left_output();
      module_output->set_rotation_current(U_(InputStates::kIs0) *
                                          drive_current_scalar);
      module_output->set_translation_current(U_(InputStates::kId0));
    }
    {
      auto module_output = output->add_front_right_output();
      module_output->set_rotation_current(U_(InputStates::kIs1) *
                                          drive_current_scalar);
      module_output->set_translation_current(U_(InputStates::kId1));
    }
    {
      auto module_output = output->add_back_left_output();
      module_output->set_rotation_current(U_(InputStates::kIs2) *
                                          drive_current_scalar);
      module_output->set_translation_current(U_(InputStates::kId2));
    }
    {
      auto module_output = output->add_back_right_output();
      module_output->set_rotation_current(U_(InputStates::kIs3) *
                                          drive_current_scalar);
      module_output->set_translation_current(U_(InputStates::kId3));
    }

    // Ignore the return value of Send
    output_builder->CheckOk(output_builder->Send());
  } else {
    U_.setZero();
    desired_heading_.reset();
  }

  if (status_builder != nullptr) {
    StatusStatic *status = status_builder->get();

    if (current_state.has_value()) {
      naive_estimator_.PopulateStatus(status->add_naive_estimator());
    }

    if (controller_result.has_value()) {
      auto controller_status = status->add_linear_controller();
      CHECK(FromEigen(controller_result->debug.goal.cast<double>(),
                      controller_status->add_goal_state()));
      CHECK(FromEigen(controller_result->debug.U_ff.cast<double>(),
                      controller_status->add_feedforwards_currents()));
      CHECK(FromEigen(controller_result->debug.U_feedback.cast<double>(),
                      controller_status->add_feedback_currents()));
      CHECK(FromEigen(
          controller_result->debug.feedback_contributions.cast<double>(),
          controller_status->add_feedback_contributions()));
      controller_status->set_sb02od_result(
          controller_result->debug.sb02od_exit_code);
    }

    imu_zeroer_.PopulateStatus(status->add_imu_zeroer());

    status->set_monotonic_timestamp_ns(
        event_loop()
            ->context()
            .monotonic_event_time.time_since_epoch()
            .count());

    // Ignore the return value of Send
    status_builder->CheckOk(status_builder->Send());
  }

  const aos::monotonic_clock::time_point sends_done =
      aos::monotonic_clock::now();

  VLOG(1) << "Loop took "
          << aos::time::DurationInSeconds(sends_done - profiling_start_time)
          << " sec total. "
          << aos::time::DurationInSeconds(estimation_done -
                                          profiling_start_time)
          << " sec in estimation "
          << aos::time::DurationInSeconds(controller_done - estimation_done)
          << " in controller "
          << aos::time::DurationInSeconds(sends_done - controller_done)
          << " sec cleaning up";
}

}  // namespace frc971::control_loops::swerve
