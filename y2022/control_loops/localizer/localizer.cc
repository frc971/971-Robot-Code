#include "y2022/control_loops/localizer/localizer.h"

#include "frc971/control_loops/c2d.h"
#include "frc971/wpilib/imu_batch_generated.h"
#include "y2022/constants.h"

namespace frc971::controls {

namespace {
constexpr double kG = 9.80665;
constexpr std::chrono::microseconds kNominalDt(500);

template <int N>
Eigen::Matrix<double, N, 1> MakeState(std::vector<double> values) {
  CHECK_EQ(static_cast<size_t>(N), values.size());
  Eigen::Matrix<double, N, 1> vector;
  for (int ii = 0; ii < N; ++ii) {
    vector(ii, 0) = values[ii];
  }
  return vector;
}
}  // namespace

ModelBasedLocalizer::ModelBasedLocalizer(
    const control_loops::drivetrain::DrivetrainConfig<double> &dt_config)
    : dt_config_(dt_config),
      velocity_drivetrain_coefficients_(
          dt_config.make_hybrid_drivetrain_velocity_loop()
              .plant()
              .coefficients()),
      down_estimator_(dt_config) {
  CHECK_EQ(branches_.capacity(), static_cast<size_t>(std::chrono::seconds(1) /
                                                 kNominalDt / kBranchPeriod));
  if (dt_config_.is_simulated) {
    down_estimator_.assume_perfect_gravity();
  }
  A_continuous_accel_.setZero();
  A_continuous_model_.setZero();
  B_continuous_accel_.setZero();
  B_continuous_model_.setZero();

  A_continuous_accel_(kX, kVelocityX) = 1.0;
  A_continuous_accel_(kY, kVelocityY) = 1.0;

  const double diameter = 2.0 * dt_config_.robot_radius;

  A_continuous_model_(kTheta, kLeftVelocity) = -1.0 / diameter;
  A_continuous_model_(kTheta, kRightVelocity) = 1.0 / diameter;
  A_continuous_model_(kLeftEncoder, kLeftVelocity) = 1.0;
  A_continuous_model_(kRightEncoder, kRightVelocity) = 1.0;
  const auto &vel_coefs = velocity_drivetrain_coefficients_;
  A_continuous_model_(kLeftVelocity, kLeftVelocity) =
      vel_coefs.A_continuous(0, 0);
  A_continuous_model_(kLeftVelocity, kRightVelocity) =
      vel_coefs.A_continuous(0, 1);
  A_continuous_model_(kRightVelocity, kLeftVelocity) =
      vel_coefs.A_continuous(1, 0);
  A_continuous_model_(kRightVelocity, kRightVelocity) =
      vel_coefs.A_continuous(1, 1);

  A_continuous_model_(kLeftVelocity, kLeftVoltageError) =
      1 * vel_coefs.B_continuous(0, 0);
  A_continuous_model_(kLeftVelocity, kRightVoltageError) =
      1 * vel_coefs.B_continuous(0, 1);
  A_continuous_model_(kRightVelocity, kLeftVoltageError) =
      1 * vel_coefs.B_continuous(1, 0);
  A_continuous_model_(kRightVelocity, kRightVoltageError) =
      1 * vel_coefs.B_continuous(1, 1);

  B_continuous_model_.block<1, 2>(kLeftVelocity, kLeftVoltage) =
      vel_coefs.B_continuous.row(0);
  B_continuous_model_.block<1, 2>(kRightVelocity, kLeftVoltage) =
      vel_coefs.B_continuous.row(1);

  B_continuous_accel_(kVelocityX, kAccelX) = 1.0;
  B_continuous_accel_(kVelocityY, kAccelY) = 1.0;
  B_continuous_accel_(kTheta, kThetaRate) = 1.0;

  Q_continuous_model_.setZero();
  Q_continuous_model_.diagonal() << 1e-4, 1e-4, 1e-4, 1e-2, 1e-0, 1e-0, 1e-2,
      1e-0, 1e-0;

  P_model_ = Q_continuous_model_ * aos::time::DurationInSeconds(kNominalDt);
}

Eigen::Matrix<double, ModelBasedLocalizer::kNModelStates,
              ModelBasedLocalizer::kNModelStates>
ModelBasedLocalizer::AModel(
    const ModelBasedLocalizer::ModelState &state) const {
  Eigen::Matrix<double, kNModelStates, kNModelStates> A = A_continuous_model_;
  const double theta = state(kTheta);
  const double stheta = std::sin(theta);
  const double ctheta = std::cos(theta);
  const double velocity = (state(kLeftVelocity) + state(kRightVelocity)) / 2.0;
  A(kX, kTheta) = -stheta * velocity;
  A(kX, kLeftVelocity) = ctheta / 2.0;
  A(kX, kRightVelocity) = ctheta / 2.0;
  A(kY, kTheta) = ctheta * velocity;
  A(kY, kLeftVelocity) = stheta / 2.0;
  A(kY, kRightVelocity) = stheta / 2.0;
  return A;
}

Eigen::Matrix<double, ModelBasedLocalizer::kNAccelStates,
              ModelBasedLocalizer::kNAccelStates>
ModelBasedLocalizer::AAccel() const {
  return A_continuous_accel_;
}

ModelBasedLocalizer::ModelState ModelBasedLocalizer::DiffModel(
    const ModelBasedLocalizer::ModelState &state,
    const ModelBasedLocalizer::ModelInput &U) const {
  ModelState x_dot = AModel(state) * state + B_continuous_model_ * U;
  const double theta = state(kTheta);
  const double stheta = std::sin(theta);
  const double ctheta = std::cos(theta);
  const double velocity = (state(kLeftVelocity) + state(kRightVelocity)) / 2.0;
  x_dot(kX) = ctheta * velocity;
  x_dot(kY) = stheta * velocity;
  return x_dot;
}

ModelBasedLocalizer::AccelState ModelBasedLocalizer::DiffAccel(
    const ModelBasedLocalizer::AccelState &state,
    const ModelBasedLocalizer::AccelInput &U) const {
  return AAccel() * state + B_continuous_accel_ * U;
}

ModelBasedLocalizer::ModelState ModelBasedLocalizer::UpdateModel(
    const ModelBasedLocalizer::ModelState &model,
    const ModelBasedLocalizer::ModelInput &input,
    const aos::monotonic_clock::duration dt) const {
  return control_loops::RungeKutta(
      std::bind(&ModelBasedLocalizer::DiffModel, this, std::placeholders::_1,
                input),
      model, aos::time::DurationInSeconds(dt));
}

ModelBasedLocalizer::AccelState ModelBasedLocalizer::UpdateAccel(
    const ModelBasedLocalizer::AccelState &accel,
    const ModelBasedLocalizer::AccelInput &input,
    const aos::monotonic_clock::duration dt) const {
  return control_loops::RungeKutta(
      std::bind(&ModelBasedLocalizer::DiffAccel, this, std::placeholders::_1,
                input),
      accel, aos::time::DurationInSeconds(dt));
}

ModelBasedLocalizer::AccelState ModelBasedLocalizer::AccelStateForModelState(
    const ModelBasedLocalizer::ModelState &state) const {
  const double robot_speed =
      (state(kLeftVelocity) + state(kRightVelocity)) / 2.0;
  const double lat_speed = (AModel(state) * state)(kTheta) * long_offset_;
  const double velocity_x = std::cos(state(kTheta)) * robot_speed -
                            std::sin(state(kTheta)) * lat_speed;
  const double velocity_y = std::sin(state(kTheta)) * robot_speed +
                            std::cos(state(kTheta)) * lat_speed;
  return (AccelState() << state(0), state(1), state(2), velocity_x, velocity_y)
      .finished();
}

ModelBasedLocalizer::ModelState ModelBasedLocalizer::ModelStateForAccelState(
    const ModelBasedLocalizer::AccelState &state,
    const Eigen::Vector2d &encoders, const double yaw_rate) const {
  const double robot_speed = state(kVelocityX) * std::cos(state(kTheta)) +
                             state(kVelocityY) * std::sin(state(kTheta));
  const double radius = dt_config_.robot_radius;
  const double left_velocity = robot_speed - yaw_rate * radius;
  const double right_velocity = robot_speed + yaw_rate * radius;
  return (ModelState() << state(0), state(1), state(2), encoders(0),
          left_velocity, 0.0, encoders(1), right_velocity, 0.0)
      .finished();
}

double ModelBasedLocalizer::ModelDivergence(
    const ModelBasedLocalizer::CombinedState &state,
    const ModelBasedLocalizer::AccelInput &accel_inputs,
    const Eigen::Vector2d &filtered_accel,
    const ModelBasedLocalizer::ModelInput &model_inputs) {
  // Convert the model state into the acceleration-based state-space and check
  // the distance between the two (should really be a weighted norm, but all the
  // numbers are on ~the same scale).
  // TODO(james): Maybe weight lateral velocity divergence different than
  // longitudinal? Seems like we tend to get false-positives currently when in
  // sharp turns.
  // TODO(james): For off-center gyros, maybe reduce noise when turning?
  VLOG(2) << "divergence: "
          << (state.accel_state - AccelStateForModelState(state.model_state))
                 .transpose();
  const AccelState diff_accel = DiffAccel(state.accel_state, accel_inputs);
  const ModelState diff_model = DiffModel(state.model_state, model_inputs);
  const double model_lng_velocity =
      (state.model_state(kLeftVelocity) + state.model_state(kRightVelocity)) /
      2.0;
  const double model_lng_accel =
      (diff_model(kLeftVelocity) + diff_model(kRightVelocity)) / 2.0 -
      diff_model(kTheta) * diff_model(kTheta) * long_offset_;
  const double model_lat_accel = diff_model(kTheta) * model_lng_velocity;
  const Eigen::Vector2d robot_frame_accel(model_lng_accel, model_lat_accel);
  const Eigen::Vector2d model_accel =
      Eigen::AngleAxisd(state.model_state(kTheta), Eigen::Vector3d::UnitZ())
          .toRotationMatrix()
          .block<2, 2>(0, 0) *
      robot_frame_accel;
  const double accel_diff = (model_accel - filtered_accel).norm();
  const double theta_rate_diff =
      std::abs(diff_accel(kTheta) - diff_model(kTheta));

  const Eigen::Vector2d accel_vel = state.accel_state.bottomRows<2>();
  Eigen::Vector2d model_vel =
      AccelStateForModelState(state.model_state).bottomRows<2>();
  velocity_residual_ = (accel_vel - model_vel).norm() /
                       (1.0 + accel_vel.norm() + model_vel.norm());
  theta_rate_residual_ = theta_rate_diff;
  accel_residual_ = accel_diff / 4.0;
  return velocity_residual_ + theta_rate_residual_ + accel_residual_;
}

void ModelBasedLocalizer::UpdateState(
    CombinedState *state,
    const Eigen::Matrix<double, kNModelStates, kNModelOutputs> &K,
    const Eigen::Matrix<double, kNModelOutputs, 1> &Z,
    const Eigen::Matrix<double, kNModelOutputs, kNModelStates> &H,
    const AccelInput &accel_input, const ModelInput &model_input,
    aos::monotonic_clock::duration dt) {
  state->accel_state = UpdateAccel(state->accel_state, accel_input, dt);
  if (down_estimator_.consecutive_still() > 500.0) {
    state->accel_state(kVelocityX) *= 0.9;
    state->accel_state(kVelocityY) *= 0.9;
  }
  state->model_state = UpdateModel(state->model_state, model_input, dt);
  state->model_state += K * (Z - H * state->model_state);
}

void ModelBasedLocalizer::HandleImu(aos::monotonic_clock::time_point t,
                                    const Eigen::Vector3d &gyro,
                                    const Eigen::Vector3d &accel,
                                    const Eigen::Vector2d encoders,
                                    const Eigen::Vector2d voltage) {
  VLOG(2) << t;
  if (t_ == aos::monotonic_clock::min_time) {
    t_ = t;
  }
  if (t_ + 2 * kNominalDt < t) {
    t_ = t;
    ++clock_resets_;
  }
  const aos::monotonic_clock::duration dt = t - t_;
  t_ = t;
  down_estimator_.Predict(gyro, accel, dt);
  // TODO(james): Should we prefer this or use the down-estimator corrected
  // version? Using the down estimator is more principled, but does create more
  // opportunities for subtle biases.
  const double yaw_rate = (dt_config_.imu_transform * gyro)(2);
  const double diameter = 2.0 * dt_config_.robot_radius;

  const Eigen::AngleAxis<double> orientation(
      Eigen::AngleAxis<double>(xytheta()(kTheta), Eigen::Vector3d::UnitZ()) *
      down_estimator_.X_hat());

  const Eigen::Vector3d absolute_accel =
      orientation * dt_config_.imu_transform * kG * accel;
  abs_accel_ = absolute_accel;

  VLOG(2) << "abs accel " << absolute_accel.transpose();
  VLOG(2) << "dt " << aos::time::DurationInSeconds(dt);

  // Update all the branched states.
  const AccelInput accel_input(absolute_accel.x(), absolute_accel.y(),
                               yaw_rate);
  const ModelInput model_input(voltage);

  const Eigen::Matrix<double, kNModelStates, kNModelStates> A_continuous =
      AModel(current_state_.model_state);

  Eigen::Matrix<double, kNModelStates, kNModelStates> A_discrete;
  Eigen::Matrix<double, kNModelStates, kNModelStates> Q_discrete;

  DiscretizeQAFast(Q_continuous_model_, A_continuous, dt, &Q_discrete,
                   &A_discrete);

  P_model_ = A_discrete * P_model_ * A_discrete.transpose() + Q_discrete;

  Eigen::Matrix<double, kNModelOutputs, kNModelStates> H;
  Eigen::Matrix<double, kNModelOutputs, kNModelOutputs> R;
  {
    H.setZero();
    R.setZero();
    H(0, kLeftEncoder) = 1.0;
    H(1, kRightEncoder) = 1.0;
    H(2, kRightVelocity) = 1.0 / diameter;
    H(2, kLeftVelocity) = -1.0 / diameter;

    R.diagonal() << 1e-9, 1e-9, 1e-13;
  }

  const Eigen::Matrix<double, kNModelOutputs, 1> Z(encoders(0), encoders(1),
                                                   yaw_rate);

  if (branches_.empty()) {
    VLOG(2) << "Initializing";
    current_state_.model_state.setZero();
    current_state_.accel_state.setZero();
    current_state_.model_state(kLeftEncoder) = encoders(0);
    current_state_.model_state(kRightEncoder) = encoders(1);
    current_state_.branch_time = t;
    branches_.Push(current_state_);
  }

  const Eigen::Matrix<double, kNModelStates, kNModelOutputs> K =
      P_model_ * H.transpose() * (H * P_model_ * H.transpose() + R).inverse();
  P_model_ = (Eigen::Matrix<double, kNModelStates, kNModelStates>::Identity() -
              K * H) *
             P_model_;
  VLOG(2) << "K\n" << K;
  VLOG(2) << "Z\n" << Z.transpose();

  for (CombinedState &state : branches_) {
    UpdateState(&state, K, Z, H, accel_input, model_input, dt);
  }
  UpdateState(&current_state_, K, Z, H, accel_input, model_input, dt);

  VLOG(2) << "oildest accel " << branches_[0].accel_state.transpose();
  VLOG(2) << "oildest accel diff "
          << DiffAccel(branches_[0].accel_state, accel_input).transpose();
  VLOG(2) << "oildest model " << branches_[0].model_state.transpose();

  // Determine whether to switch modes--if we are currently in model-based mode,
  // swap to accel-based if the two states have divergeed meaningfully in the
  // oldest branch. If we are currently in accel-based, then swap back to model
  // if the oldest model branch matches has matched the
  filtered_residual_accel_ +=
      0.01 * (accel_input.topRows<2>() - filtered_residual_accel_);
  const double model_divergence =
      branches_.full() ? ModelDivergence(branches_[0], accel_input,
                                         filtered_residual_accel_, model_input)
                       : 0.0;
  filtered_residual_ +=
      (1.0 - std::exp(-aos::time::DurationInSeconds(kNominalDt) / 0.0095)) *
      (model_divergence - filtered_residual_);
  // TODO(james): Tune this more. Currently set to generally trust the model,
  // perhaps a bit too much.
  // When the residual exceeds the accel threshold, we start using the inertials
  // alone; when it drops back below the model threshold, we go back to being
  // model-based.
  constexpr double kUseAccelThreshold = 2.0;
  constexpr double kUseModelThreshold = 0.5;
  constexpr size_t kShareStates = kNModelStates;
  static_assert(kUseModelThreshold < kUseAccelThreshold);
  if (using_model_) {
    if (filtered_residual_ > kUseAccelThreshold) {
      hysteresis_count_++;
    } else {
      hysteresis_count_ = 0;
    }
    if (hysteresis_count_ > 0) {
      using_model_ = false;
      // Grab the accel-based state from back when we started diverging.
      // TODO(james): This creates a problematic selection bias, because
      // we will tend to bias towards deliberately out-of-tune measurements.
      current_state_.accel_state = branches_[0].accel_state;
      current_state_.model_state = branches_[0].model_state;
      current_state_.model_state = ModelStateForAccelState(
          current_state_.accel_state, encoders, yaw_rate);
    } else {
      VLOG(2) << "Normal branching";
      current_state_.accel_state =
          AccelStateForModelState(current_state_.model_state);
      current_state_.branch_time = t;
    }
    hysteresis_count_ = 0;
  } else {
    if (filtered_residual_ < kUseModelThreshold) {
      hysteresis_count_++;
    } else {
      hysteresis_count_ = 0;
    }
    if (hysteresis_count_ > 100) {
      using_model_ = true;
      // Grab the model-based state from back when we stopped diverging.
      current_state_.model_state.topRows<kShareStates>() =
          ModelStateForAccelState(branches_[0].accel_state, encoders, yaw_rate)
              .topRows<kShareStates>();
      current_state_.accel_state =
          AccelStateForModelState(current_state_.model_state);
    } else {
      // TODO(james): Why was I leaving the encoders/wheel velocities in place?
      current_state_.model_state = ModelStateForAccelState(
          current_state_.accel_state, encoders, yaw_rate);
      current_state_.branch_time = t;
    }
  }

  // Generate a new branch, with the accel state reset based on the model-based
  // state (really, just getting rid of the lateral velocity).
  // By resetting the accel state in the new branch, this tries to minimize the
  // odds of runaway lateral velocities. This doesn't help with runaway
  // longitudinal velocities, however.
  CombinedState new_branch = current_state_;
  new_branch.accel_state = AccelStateForModelState(new_branch.model_state);
  new_branch.accumulated_divergence = 0.0;

  ++branch_counter_;
  if (branch_counter_ % kBranchPeriod == 0) {
    branches_.Push(new_branch);
    branch_counter_ = 0;
  }

  last_residual_ = model_divergence;

  VLOG(2) << "Using " << (using_model_ ? "model" : "accel");
  VLOG(2) << "Residual " << last_residual_;
  VLOG(2) << "Filtered Residual " << filtered_residual_;
  VLOG(2) << "buffer size " << branches_.size();
  VLOG(2) << "Model state " << current_state_.model_state.transpose();
  VLOG(2) << "Accel state " << current_state_.accel_state.transpose();
  VLOG(2) << "Accel state for model "
            << AccelStateForModelState(current_state_.model_state).transpose();
  VLOG(2) << "Input acce " << accel.transpose();
  VLOG(2) << "Input gyro " << gyro.transpose();
  VLOG(2) << "Input voltage " << voltage.transpose();
  VLOG(2) << "Input encoder " << encoders.transpose();
  VLOG(2) << "yaw rate " << yaw_rate;

  CHECK(std::isfinite(last_residual_));
}

void ModelBasedLocalizer::HandleReset(aos::monotonic_clock::time_point now,
                                      const Eigen::Vector3d &xytheta) {
  branches_.Reset();
  t_ =  now;
  using_model_ = true;
  current_state_.model_state << xytheta(0), xytheta(1), xytheta(2),
      current_state_.model_state(kLeftEncoder), 0.0, 0.0,
      current_state_.model_state(kRightEncoder), 0.0, 0.0;
  current_state_.accel_state =
      AccelStateForModelState(current_state_.model_state);
  last_residual_ = 0.0;
  filtered_residual_ = 0.0;
  filtered_residual_accel_.setZero();
  abs_accel_.setZero();
}

flatbuffers::Offset<AccelBasedState> ModelBasedLocalizer::BuildAccelState(
    flatbuffers::FlatBufferBuilder *fbb, const AccelState &state) {
  AccelBasedState::Builder accel_state_builder(*fbb);
  accel_state_builder.add_x(state(kX));
  accel_state_builder.add_y(state(kY));
  accel_state_builder.add_theta(state(kTheta));
  accel_state_builder.add_velocity_x(state(kVelocityX));
  accel_state_builder.add_velocity_y(state(kVelocityY));
  return accel_state_builder.Finish();
}

flatbuffers::Offset<ModelBasedState> ModelBasedLocalizer::BuildModelState(
    flatbuffers::FlatBufferBuilder *fbb, const ModelState &state) {
  ModelBasedState::Builder model_state_builder(*fbb);
  model_state_builder.add_x(state(kX));
  model_state_builder.add_y(state(kY));
  model_state_builder.add_theta(state(kTheta));
  model_state_builder.add_left_encoder(state(kLeftEncoder));
  model_state_builder.add_left_velocity(state(kLeftVelocity));
  model_state_builder.add_left_voltage_error(state(kLeftVoltageError));
  model_state_builder.add_right_encoder(state(kRightEncoder));
  model_state_builder.add_right_velocity(state(kRightVelocity));
  model_state_builder.add_right_voltage_error(state(kRightVoltageError));
  return model_state_builder.Finish();
}

flatbuffers::Offset<ModelBasedStatus> ModelBasedLocalizer::PopulateStatus(
    flatbuffers::FlatBufferBuilder *fbb) {
  const flatbuffers::Offset<control_loops::drivetrain::DownEstimatorState>
      down_estimator_offset = down_estimator_.PopulateStatus(fbb, t_);

  const CombinedState &state = current_state_;

  const flatbuffers::Offset<ModelBasedState> model_state_offset =
    BuildModelState(fbb, state.model_state);

  const flatbuffers::Offset<AccelBasedState> accel_state_offset =
      BuildAccelState(fbb, state.accel_state);

  const flatbuffers::Offset<AccelBasedState> oldest_accel_state_offset =
      branches_.empty() ? flatbuffers::Offset<AccelBasedState>()
                        : BuildAccelState(fbb, branches_[0].accel_state);

  const flatbuffers::Offset<ModelBasedState> oldest_model_state_offset =
      branches_.empty() ? flatbuffers::Offset<ModelBasedState>()
                        : BuildModelState(fbb, branches_[0].model_state);

  ModelBasedStatus::Builder builder(*fbb);
  builder.add_accel_state(accel_state_offset);
  builder.add_oldest_accel_state(oldest_accel_state_offset);
  builder.add_oldest_model_state(oldest_model_state_offset);
  builder.add_model_state(model_state_offset);
  builder.add_using_model(using_model_);
  builder.add_residual(last_residual_);
  builder.add_filtered_residual(filtered_residual_);
  builder.add_velocity_residual(velocity_residual_);
  builder.add_accel_residual(accel_residual_);
  builder.add_theta_rate_residual(theta_rate_residual_);
  builder.add_down_estimator(down_estimator_offset);
  builder.add_x(xytheta()(0));
  builder.add_y(xytheta()(1));
  builder.add_theta(xytheta()(2));
  builder.add_implied_accel_x(abs_accel_(0));
  builder.add_implied_accel_y(abs_accel_(1));
  builder.add_implied_accel_z(abs_accel_(2));
  builder.add_clock_resets(clock_resets_);
  return builder.Finish();
}

namespace {
// Period at which the encoder readings from the IMU board wrap.
static double DrivetrainWrapPeriod() {
  return y2022::constants::Values::DrivetrainEncoderToMeters(1 << 16);
}
}

EventLoopLocalizer::EventLoopLocalizer(
    aos::EventLoop *event_loop,
    const control_loops::drivetrain::DrivetrainConfig<double> &dt_config)
    : event_loop_(event_loop),
      model_based_(dt_config),
      status_sender_(event_loop_->MakeSender<LocalizerStatus>("/localizer")),
      output_sender_(event_loop_->MakeSender<LocalizerOutput>("/localizer")),
      output_fetcher_(
          event_loop_->MakeFetcher<frc971::control_loops::drivetrain::Output>(
              "/drivetrain")),
      left_encoder_(-DrivetrainWrapPeriod() / 2.0, DrivetrainWrapPeriod()),
      right_encoder_(-DrivetrainWrapPeriod() / 2.0, DrivetrainWrapPeriod()) {
  event_loop_->MakeWatcher(
      "/drivetrain",
      [this](
          const frc971::control_loops::drivetrain::LocalizerControl &control) {
        const double theta = control.keep_current_theta()
                                 ? model_based_.xytheta()(2)
                                 : control.theta();
        model_based_.HandleReset(event_loop_->monotonic_now(),
                               {control.x(), control.y(), theta});
      });
  event_loop_->MakeWatcher(
      "/localizer", [this](const frc971::IMUValuesBatch &values) {
        CHECK(values.has_readings());
        output_fetcher_.Fetch();
        for (const IMUValues *value : *values.readings()) {
          zeroer_.InsertAndProcessMeasurement(*value);
          const Eigen::Vector2d encoders{
              left_encoder_.Unwrap(value->left_encoder()),
              right_encoder_.Unwrap(value->right_encoder())};
          if (zeroer_.Zeroed()) {
            const aos::monotonic_clock::time_point pico_timestamp{
                std::chrono::microseconds(value->pico_timestamp_us())};
            // TODO(james): If we get large enough drift off of the pico,
            // actually do something about it.
            if (!pico_offset_.has_value()) {
              pico_offset_ =
                  event_loop_->context().monotonic_event_time - pico_timestamp;
              last_pico_timestamp_ = pico_timestamp;
            }
            if (pico_timestamp < last_pico_timestamp_) {
              pico_offset_.value() += std::chrono::microseconds(1ULL << 32);
            }
            const aos::monotonic_clock::time_point sample_timestamp =
                pico_offset_.value() + pico_timestamp;
            pico_offset_error_ =
                event_loop_->context().monotonic_event_time - sample_timestamp;
            const bool disabled =
                (output_fetcher_.get() == nullptr) ||
                (output_fetcher_.context().monotonic_event_time +
                     std::chrono::milliseconds(10) <
                 event_loop_->context().monotonic_event_time);
            model_based_.HandleImu(
                sample_timestamp,
                zeroer_.ZeroedGyro(), zeroer_.ZeroedAccel(), encoders,
                disabled ? Eigen::Vector2d::Zero()
                         : Eigen::Vector2d{output_fetcher_->left_voltage(),
                                           output_fetcher_->right_voltage()});
            last_pico_timestamp_ = pico_timestamp;
          }
          {
            auto builder = status_sender_.MakeBuilder();
            const flatbuffers::Offset<ModelBasedStatus> model_based_status =
                model_based_.PopulateStatus(builder.fbb());
            const flatbuffers::Offset<control_loops::drivetrain::ImuZeroerState>
                zeroer_status = zeroer_.PopulateStatus(builder.fbb());
            LocalizerStatus::Builder status_builder =
                builder.MakeBuilder<LocalizerStatus>();
            status_builder.add_model_based(model_based_status);
            status_builder.add_zeroed(zeroer_.Zeroed());
            status_builder.add_faulted_zero(zeroer_.Faulted());
            status_builder.add_zeroing(zeroer_status);
            status_builder.add_left_encoder(encoders(0));
            status_builder.add_right_encoder(encoders(1));
            if (pico_offset_.has_value()) {
              status_builder.add_pico_offset_ns(pico_offset_.value().count());
              status_builder.add_pico_offset_error_ns(
                  pico_offset_error_.count());
            }
            builder.CheckOk(builder.Send(status_builder.Finish()));
          }
          if (last_output_send_ + std::chrono::milliseconds(5) <
              event_loop_->context().monotonic_event_time) {
            auto builder = output_sender_.MakeBuilder();
            LocalizerOutput::Builder output_builder =
                builder.MakeBuilder<LocalizerOutput>();
            // TODO(james): Should we bother to try to estimate time offsets for
            // the pico?
            output_builder.add_monotonic_timestamp_ns(
                value->monotonic_timestamp_ns());
            output_builder.add_x(model_based_.xytheta()(0));
            output_builder.add_y(model_based_.xytheta()(1));
            output_builder.add_theta(model_based_.xytheta()(2));
            builder.CheckOk(builder.Send(output_builder.Finish()));
            last_output_send_ = event_loop_->monotonic_now();
          }
        }
      });
}

}  // namespace frc971::controls
