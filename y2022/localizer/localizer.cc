#include "y2022/localizer/localizer.h"

#include "aos/json_to_flatbuffer.h"
#include "frc971/control_loops/c2d.h"
#include "frc971/wpilib/imu_batch_generated.h"
#include "y2022/constants.h"

DEFINE_bool(ignore_accelerometer, false,
            "If set, ignores the accelerometer readings.");

namespace frc971::controls {

namespace {
constexpr double kG = 9.80665;
static constexpr std::chrono::microseconds kNominalDt = ImuWatcher::kNominalDt;
// Field position of the target (the 2022 target is conveniently in the middle
// of the field....).
constexpr double kVisionTargetX = 0.0;
constexpr double kVisionTargetY = 0.0;

// Minimum confidence to require to use a target match.
constexpr double kMinTargetEstimateConfidence = 0.75;

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
  statistics_.rejection_counts.fill(0);
  CHECK_EQ(branches_.capacity(),
           static_cast<size_t>(std::chrono::seconds(1) / kNominalDt /
                               kBranchPeriod));
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
  Q_continuous_model_.diagonal() << 1e-2, 1e-2, 1e-8, 1e-2, 1e-0, 1e-0, 1e-2,
      1e-0, 1e-0;

  Q_continuous_accel_.setZero();
  Q_continuous_accel_.diagonal() << 1e-2, 1e-2, 1e-20, 1e-4, 1e-4;

  P_model_ = Q_continuous_model_ * aos::time::DurationInSeconds(kNominalDt);

  // We can precalculate the discretizations of the accel model because it is
  // actually LTI.

  DiscretizeQAFast(Q_continuous_accel_, A_continuous_accel_, kNominalDt,
                   &Q_discrete_accel_, &A_discrete_accel_);
  P_accel_ = Q_discrete_accel_;

  led_outputs_.fill(LedOutput::ON);
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
  const double lat_speed = (AModel(state) * state)(kTheta)*long_offset_;
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

void ModelBasedLocalizer::HandleImu(
    aos::monotonic_clock::time_point t, const Eigen::Vector3d &gyro,
    const Eigen::Vector3d &accel, const std::optional<Eigen::Vector2d> encoders,
    const Eigen::Vector2d voltage) {
  VLOG(2) << t;
  if (t_ == aos::monotonic_clock::min_time) {
    t_ = t;
  }
  if (t_ + 10 * kNominalDt < t) {
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
  last_orientation_ = orientation;

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
  P_accel_ = A_discrete_accel_ * P_accel_ * A_discrete_accel_.transpose() +
             Q_discrete_accel_;

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

  const Eigen::Matrix<double, kNModelOutputs, 1> Z =
      encoders.has_value()
          ? Eigen::Vector3d(encoders.value()(0), encoders.value()(1), yaw_rate)
          : Eigen::Vector3d(current_state_.model_state(kLeftEncoder),
                            current_state_.model_state(kRightEncoder),
                            yaw_rate);

  if (branches_.empty()) {
    VLOG(2) << "Initializing";
    current_state_.model_state(kLeftEncoder) = Z(0);
    current_state_.model_state(kRightEncoder) = Z(1);
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
    if (!FLAGS_ignore_accelerometer &&
        filtered_residual_ > kUseAccelThreshold) {
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
          current_state_.accel_state, Z.topRows<2>(), yaw_rate);
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
          ModelStateForAccelState(branches_[0].accel_state, Z.topRows<2>(),
                                  yaw_rate)
              .topRows<kShareStates>();
      current_state_.accel_state =
          AccelStateForModelState(current_state_.model_state);
    } else {
      // TODO(james): Why was I leaving the encoders/wheel velocities in place?
      current_state_.model_state = ModelStateForAccelState(
          current_state_.accel_state, Z.topRows<2>(), yaw_rate);
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
    old_positions_.Push(OldPosition{t, xytheta(), latest_turret_position_,
                                    latest_turret_velocity_});
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
  VLOG(2) << "Input encoder " << Z.topRows<2>().transpose();
  VLOG(2) << "yaw rate " << yaw_rate;

  CHECK(std::isfinite(last_residual_));
}

const ModelBasedLocalizer::OldPosition ModelBasedLocalizer::GetStateForTime(
    aos::monotonic_clock::time_point time) {
  if (old_positions_.empty()) {
    return OldPosition{};
  }

  aos::monotonic_clock::duration lowest_time_error =
      aos::monotonic_clock::duration::max();
  const OldPosition *best_match = nullptr;
  for (const OldPosition &sample : old_positions_) {
    const aos::monotonic_clock::duration time_error =
        std::chrono::abs(sample.sample_time - time);
    if (time_error < lowest_time_error) {
      lowest_time_error = time_error;
      best_match = &sample;
    }
  }
  return *best_match;
}

namespace {

// Node names of the pis to listen for cameras from.
constexpr std::array<std::string_view, ModelBasedLocalizer::kNumPis> kPisToUse{
    "pi1", "pi2", "pi3", "pi4"};
}  // namespace

const Eigen::Matrix<double, 4, 4> ModelBasedLocalizer::CameraTransform(
    const OldPosition &state,
    const frc971::vision::calibration::CameraCalibration *calibration,
    std::optional<RejectionReason> *rejection_reason) const {
  CHECK_NOTNULL(rejection_reason);
  CHECK_NOTNULL(calibration);
  // Per the CameraCalibration specification, we can actually determine whether
  // the camera is the turret camera just from the presence of the
  // turret_extrinsics member.
  const bool is_turret = calibration->has_turret_extrinsics();
  // Ignore readings when the turret is spinning too fast, on the assumption
  // that the odds of screwing up the time compensation are higher.
  // Note that the current number here is chosen pretty arbitrarily--1 rad / sec
  // seems reasonable, but may be unnecessarily low or high.
  constexpr double kMaxTurretVelocity = 1.0;
  if (is_turret && std::abs(state.turret_velocity) > kMaxTurretVelocity &&
      !rejection_reason->has_value()) {
    *rejection_reason = RejectionReason::TURRET_TOO_FAST;
  }
  CHECK(calibration->has_fixed_extrinsics());
  const Eigen::Matrix<double, 4, 4> fixed_extrinsics =
      control_loops::drivetrain::FlatbufferToTransformationMatrix(
          *calibration->fixed_extrinsics());

  // Calculate the pose of the camera relative to the robot origin.
  Eigen::Matrix<double, 4, 4> H_robot_camera = fixed_extrinsics;
  if (is_turret) {
    H_robot_camera =
        H_robot_camera *
        frc971::control_loops::TransformationMatrixForYaw<double>(
            state.turret_position) *
        control_loops::drivetrain::FlatbufferToTransformationMatrix(
            *calibration->turret_extrinsics());
  }
  return H_robot_camera;
}

const std::optional<Eigen::Vector2d>
ModelBasedLocalizer::CameraMeasuredRobotPosition(
    const OldPosition &state, const y2022::vision::TargetEstimate *target,
    std::optional<RejectionReason> *rejection_reason,
    Eigen::Matrix<double, 4, 4> *H_field_camera_measured) const {
  if (!target->has_camera_calibration()) {
    *rejection_reason = RejectionReason::NO_CALIBRATION;
    return std::nullopt;
  }
  const Eigen::Matrix<double, 4, 4> H_robot_camera =
      CameraTransform(state, target->camera_calibration(), rejection_reason);
  const control_loops::Pose robot_pose(
      {state.xytheta(0), state.xytheta(1), 0.0}, state.xytheta(2));
  const Eigen::Matrix<double, 4, 4> H_field_robot =
      robot_pose.AsTransformationMatrix();
  // Current estimated pose of the camera in the global frame.
  // Note that this is all really just an elaborate way of extracting the
  // current estimated camera yaw, and nothing else.
  const Eigen::Matrix<double, 4, 4> H_field_camera =
      H_field_robot * H_robot_camera;
  // Grab the implied yaw of the camera (the +Z axis is coming out of the front
  // of the cameras).
  const Eigen::Vector3d rotated_camera_z =
      H_field_camera.block<3, 3>(0, 0) * Eigen::Vector3d(0, 0, 1);
  const double camera_yaw =
      std::atan2(rotated_camera_z.y(), rotated_camera_z.x());
  // All right, now we need to use the heading and distance from the
  // TargetEstimate, plus the yaw embedded in the camera_pose, to determine what
  // the implied X/Y position of the robot is. To do this, we calculate the
  // heading/distance from the target to the robot. The distance is easy, since
  // that's the same as the distance from the robot to the target. The heading
  // isn't too hard, but is obnoxious to think about, since the heading from the
  // target to the robot is distinct from the heading from the robot to the
  // target.

  // Just to walk through examples to confirm that the below calculation is
  // correct:
  // * If yaw = 0, and angle_to_target = 0, we are at 180 deg relative to the
  //   target.
  // * If yaw = 90 deg, and angle_to_target = 0, we are at -90 deg relative to
  //   the target.
  // * If yaw = 0, and angle_to_target = 90 deg, we are at -90 deg relative to
  //   the target.
  const double heading_from_target =
      aos::math::NormalizeAngle(M_PI + camera_yaw + target->angle_to_target());
  const double distance_from_target = target->distance();
  // Extract the implied camera position on the field.
  *H_field_camera_measured = H_field_camera;
  // TODO(james): Are we going to need to evict the roll/pitch components of the
  // camera extrinsics this year as well?
  (*H_field_camera_measured)(0, 3) =
      distance_from_target * std::cos(heading_from_target) + kVisionTargetX;
  (*H_field_camera_measured)(1, 3) =
      distance_from_target * std::sin(heading_from_target) + kVisionTargetY;
  const Eigen::Matrix<double, 4, 4> H_field_robot_measured =
      *H_field_camera_measured * H_robot_camera.inverse();
  return H_field_robot_measured.block<2, 1>(0, 3);
}

void ModelBasedLocalizer::HandleImageMatch(
    aos::monotonic_clock::time_point sample_time,
    const y2022::vision::TargetEstimate *target, int camera_index) {
  std::optional<RejectionReason> rejection_reason;

  if (target->confidence() < kMinTargetEstimateConfidence) {
    rejection_reason = RejectionReason::LOW_CONFIDENCE;
    TallyRejection(rejection_reason.value());
    return;
  }

  const OldPosition &state = GetStateForTime(sample_time);
  Eigen::Matrix<double, 4, 4> H_field_camera_measured;
  const std::optional<Eigen::Vector2d> measured_robot_position =
      CameraMeasuredRobotPosition(state, target, &rejection_reason,
                                  &H_field_camera_measured);
  // Technically, rejection_reason should always be set if
  // measured_robot_position is nullopt, but in the future we may have more
  // recoverable rejection reasons that we wish to allow to propagate further
  // into the process.
  if (!measured_robot_position || rejection_reason.has_value()) {
    CHECK(rejection_reason.has_value());
    TallyRejection(rejection_reason.value());
    return;
  }

  // Next, go through and do the actual Kalman corrections for the x/y
  // measurement, for both the accel state and the model-based state.
  const Eigen::Matrix<double, kNModelStates, kNModelStates> A_continuous_model =
      AModel(current_state_.model_state);

  Eigen::Matrix<double, kNModelStates, kNModelStates> A_discrete_model;
  Eigen::Matrix<double, kNModelStates, kNModelStates> Q_discrete_model;

  DiscretizeQAFast(Q_continuous_model_, A_continuous_model, kNominalDt,
                   &Q_discrete_model, &A_discrete_model);

  Eigen::Matrix<double, 2, kNModelStates> H_model;
  H_model.setZero();
  Eigen::Matrix<double, 2, kNAccelStates> H_accel;
  H_accel.setZero();
  Eigen::Matrix<double, 2, 2> R;
  R.setZero();
  H_model(0, kX) = 1.0;
  H_model(1, kY) = 1.0;
  H_accel(0, kX) = 1.0;
  H_accel(1, kY) = 1.0;
  if (aggressive_corrections_) {
    R.diagonal() << 1e-2, 1e-2;
  } else {
    R.diagonal() << 1e-0, 1e-0;
  }

  const Eigen::Matrix<double, kNModelStates, 2> K_model =
      P_model_ * H_model.transpose() *
      (H_model * P_model_ * H_model.transpose() + R).inverse();
  const Eigen::Matrix<double, kNAccelStates, 2> K_accel =
      P_accel_ * H_accel.transpose() *
      (H_accel * P_accel_ * H_accel.transpose() + R).inverse();
  P_model_ = (Eigen::Matrix<double, kNModelStates, kNModelStates>::Identity() -
              K_model * H_model) *
             P_model_;
  P_accel_ = (Eigen::Matrix<double, kNAccelStates, kNAccelStates>::Identity() -
              K_accel * H_accel) *
             P_accel_;
  // And now we have to correct *everything* on all the branches:
  for (CombinedState &state : branches_) {
    state.model_state += K_model * (measured_robot_position.value() -
                                    H_model * state.model_state);
    state.accel_state += K_accel * (measured_robot_position.value() -
                                    H_accel * state.accel_state);
  }
  current_state_.model_state +=
      K_model *
      (measured_robot_position.value() - H_model * current_state_.model_state);
  current_state_.accel_state +=
      K_accel *
      (measured_robot_position.value() - H_accel * current_state_.accel_state);

  statistics_.total_accepted++;
  statistics_.total_candidates++;

  const Eigen::Vector3d camera_z_in_field =
      H_field_camera_measured.block<3, 3>(0, 0) * Eigen::Vector3d::UnitZ();
  const double camera_yaw =
      std::atan2(camera_z_in_field.y(), camera_z_in_field.x());

  // TODO(milind): actually control this
  led_outputs_[camera_index] = LedOutput::ON;

  TargetEstimateDebugT debug;
  debug.camera = static_cast<uint8_t>(camera_index);
  debug.camera_x = H_field_camera_measured(0, 3);
  debug.camera_y = H_field_camera_measured(1, 3);
  debug.camera_theta = camera_yaw;
  debug.implied_robot_x = measured_robot_position.value().x();
  debug.implied_robot_y = measured_robot_position.value().y();
  debug.implied_robot_theta = xytheta()(2);
  debug.implied_turret_goal =
      aos::math::NormalizeAngle(camera_yaw + target->angle_to_target());
  debug.accepted = true;
  debug.image_age_sec = aos::time::DurationInSeconds(t_ - sample_time);
  CHECK_LT(image_debugs_.size(), kDebugBufferSize);
  image_debugs_.push_back(debug);
}

void ModelBasedLocalizer::HandleTurret(
    aos::monotonic_clock::time_point sample_time, double turret_position,
    double turret_velocity) {
  last_turret_update_ = sample_time;
  latest_turret_position_ = turret_position;
  latest_turret_velocity_ = turret_velocity;
}

void ModelBasedLocalizer::HandleReset(aos::monotonic_clock::time_point now,
                                      const Eigen::Vector3d &xytheta) {
  branches_.Reset();
  t_ = now;
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

flatbuffers::Offset<CumulativeStatistics>
ModelBasedLocalizer::PopulateStatistics(flatbuffers::FlatBufferBuilder *fbb) {
  const auto rejections_offset = fbb->CreateVector(
      statistics_.rejection_counts.data(), statistics_.rejection_counts.size());

  CumulativeStatistics::Builder stats_builder(*fbb);
  stats_builder.add_total_accepted(statistics_.total_accepted);
  stats_builder.add_total_candidates(statistics_.total_candidates);
  stats_builder.add_rejection_reason_count(rejections_offset);
  return stats_builder.Finish();
}

flatbuffers::Offset<ModelBasedStatus> ModelBasedLocalizer::PopulateStatus(
    flatbuffers::FlatBufferBuilder *fbb) {
  const flatbuffers::Offset<CumulativeStatistics> stats_offset =
      PopulateStatistics(fbb);

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
  builder.add_statistics(stats_offset);
  return builder.Finish();
}

flatbuffers::Offset<LocalizerVisualization>
ModelBasedLocalizer::PopulateVisualization(
    flatbuffers::FlatBufferBuilder *fbb) {
  const flatbuffers::Offset<CumulativeStatistics> stats_offset =
      PopulateStatistics(fbb);

  aos::SizedArray<flatbuffers::Offset<TargetEstimateDebug>, kDebugBufferSize>
      debug_offsets;

  for (const TargetEstimateDebugT &debug : image_debugs_) {
    debug_offsets.push_back(PackTargetEstimateDebug(debug, fbb));
  }

  image_debugs_.clear();

  const flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<TargetEstimateDebug>>>
      debug_offset =
          fbb->CreateVector(debug_offsets.data(), debug_offsets.size());

  LocalizerVisualization::Builder builder(*fbb);
  builder.add_statistics(stats_offset);
  builder.add_targets(debug_offset);
  return builder.Finish();
}

void ModelBasedLocalizer::TallyRejection(const RejectionReason reason) {
  statistics_.total_candidates++;
  statistics_.rejection_counts[static_cast<size_t>(reason)]++;
  TargetEstimateDebugT debug;
  debug.accepted = false;
  debug.rejection_reason = reason;
  CHECK_LT(image_debugs_.size(), kDebugBufferSize);
  image_debugs_.push_back(debug);
}

flatbuffers::Offset<TargetEstimateDebug>
ModelBasedLocalizer::PackTargetEstimateDebug(
    const TargetEstimateDebugT &debug, flatbuffers::FlatBufferBuilder *fbb) {
  if (!debug.accepted) {
    TargetEstimateDebug::Builder builder(*fbb);
    builder.add_accepted(debug.accepted);
    builder.add_rejection_reason(debug.rejection_reason);
    return builder.Finish();
  } else {
    flatbuffers::Offset<TargetEstimateDebug> offset =
        TargetEstimateDebug::Pack(*fbb, &debug);
    flatbuffers::GetMutableTemporaryPointer(*fbb, offset)
        ->clear_rejection_reason();
    return offset;
  }
}

EventLoopLocalizer::EventLoopLocalizer(
    aos::EventLoop *event_loop,
    const control_loops::drivetrain::DrivetrainConfig<double> &dt_config)
    : event_loop_(event_loop),
      model_based_(dt_config),
      status_sender_(event_loop_->MakeSender<LocalizerStatus>("/localizer")),
      output_sender_(event_loop_->MakeSender<LocalizerOutput>("/localizer")),
      visualization_sender_(
          event_loop_->MakeSender<LocalizerVisualization>("/localizer")),
      superstructure_fetcher_(
          event_loop_
              ->MakeFetcher<y2022::control_loops::superstructure::Status>(
                  "/superstructure")),
      imu_watcher_(event_loop, dt_config,
                   y2022::constants::Values::DrivetrainEncoderToMeters(1),
                   [this](aos::monotonic_clock::time_point sample_time_pico,
                          aos::monotonic_clock::time_point sample_time_pi,
                          std::optional<Eigen::Vector2d> encoders,
                          Eigen::Vector3d gyro, Eigen::Vector3d accel) {
                     HandleImu(sample_time_pico, sample_time_pi, encoders, gyro,
                               accel);
                   }),
      utils_(event_loop) {
  event_loop->SetRuntimeRealtimePriority(10);
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
  aos::TimerHandler *superstructure_timer = event_loop_->AddTimer([this]() {
    if (superstructure_fetcher_.Fetch()) {
      const y2022::control_loops::superstructure::Status &status =
          *superstructure_fetcher_.get();
      if (!status.has_turret()) {
        return;
      }
      CHECK(status.has_turret());
      model_based_.HandleTurret(
          superstructure_fetcher_.context().monotonic_event_time,
          status.turret()->position(), status.turret()->velocity());
    }
  });
  event_loop_->OnRun([this, superstructure_timer]() {
    superstructure_timer->Schedule(event_loop_->monotonic_now(),
                                   std::chrono::milliseconds(20));
  });

  for (size_t camera_index = 0; camera_index < kPisToUse.size();
       ++camera_index) {
    CHECK_LT(camera_index, target_estimate_fetchers_.size());
    target_estimate_fetchers_[camera_index] =
        event_loop_->MakeFetcher<y2022::vision::TargetEstimate>(
            absl::StrCat("/", kPisToUse[camera_index], "/camera"));
  }
  aos::TimerHandler *estimate_timer = event_loop_->AddTimer([this]() {
    const bool maybe_in_auto = utils_.MaybeInAutonomous();
    model_based_.set_use_aggressive_image_corrections(!maybe_in_auto);
    for (size_t camera_index = 0; camera_index < kPisToUse.size();
         ++camera_index) {
      if (model_based_.NumQueuedImageDebugs() ==
              ModelBasedLocalizer::kDebugBufferSize ||
          (last_visualization_send_ + kMinVisualizationPeriod <
           event_loop_->monotonic_now())) {
        auto builder = visualization_sender_.MakeBuilder();
        visualization_sender_.CheckOk(
            builder.Send(model_based_.PopulateVisualization(builder.fbb())));
      }
      if (target_estimate_fetchers_[camera_index].Fetch()) {
        const std::optional<aos::monotonic_clock::duration> monotonic_offset =
            utils_.ClockOffset(kPisToUse[camera_index]);
        if (!monotonic_offset.has_value()) {
          model_based_.TallyRejection(
              RejectionReason::MESSAGE_BRIDGE_DISCONNECTED);
          continue;
        }
        // TODO(james): Get timestamp from message contents.
        aos::monotonic_clock::time_point capture_time(
            target_estimate_fetchers_[camera_index]
                .context()
                .monotonic_remote_time -
            monotonic_offset.value());
        if (capture_time > target_estimate_fetchers_[camera_index]
                               .context()
                               .monotonic_event_time) {
          model_based_.TallyRejection(RejectionReason::IMAGE_FROM_FUTURE);
          continue;
        }
        capture_time -= imu_watcher_.pico_offset_error();
        model_based_.HandleImageMatch(
            capture_time, target_estimate_fetchers_[camera_index].get(),
            camera_index);
      }
    }
  });
  event_loop_->OnRun([this, estimate_timer]() {
    estimate_timer->Schedule(event_loop_->monotonic_now(),
                             std::chrono::milliseconds(100));
  });
}

void EventLoopLocalizer::HandleImu(
    aos::monotonic_clock::time_point sample_time_pico,
    aos::monotonic_clock::time_point sample_time_pi,
    std::optional<Eigen::Vector2d> encoders, Eigen::Vector3d gyro,
    Eigen::Vector3d accel) {
  model_based_.HandleImu(
      sample_time_pico, gyro, accel, encoders,
      utils_.VoltageOrZero(event_loop_->context().monotonic_event_time));

  {
    auto builder = status_sender_.MakeBuilder();
    const flatbuffers::Offset<ModelBasedStatus> model_based_status =
        model_based_.PopulateStatus(builder.fbb());
    const flatbuffers::Offset<control_loops::drivetrain::ImuZeroerState>
        zeroer_status = imu_watcher_.zeroer().PopulateStatus(builder.fbb());
    const flatbuffers::Offset<ImuFailures> imu_failures =
        imu_watcher_.PopulateImuFailures(builder.fbb());
    LocalizerStatus::Builder status_builder =
        builder.MakeBuilder<LocalizerStatus>();
    status_builder.add_model_based(model_based_status);
    status_builder.add_zeroed(imu_watcher_.zeroer().Zeroed());
    status_builder.add_faulted_zero(imu_watcher_.zeroer().Faulted());
    status_builder.add_zeroing(zeroer_status);
    status_builder.add_imu_failures(imu_failures);
    if (encoders.has_value()) {
      status_builder.add_left_encoder(encoders.value()(0));
      status_builder.add_right_encoder(encoders.value()(1));
    }
    if (imu_watcher_.pico_offset().has_value()) {
      status_builder.add_pico_offset_ns(
          imu_watcher_.pico_offset().value().count());
      status_builder.add_pico_offset_error_ns(
          imu_watcher_.pico_offset_error().count());
    }
    builder.CheckOk(builder.Send(status_builder.Finish()));
  }
  if (last_output_send_ + std::chrono::milliseconds(5) <
      event_loop_->context().monotonic_event_time) {
    auto builder = output_sender_.MakeBuilder();

    const auto led_outputs_offset = builder.fbb()->CreateVector(
        model_based_.led_outputs().data(), model_based_.led_outputs().size());

    LocalizerOutput::Builder output_builder =
        builder.MakeBuilder<LocalizerOutput>();
    output_builder.add_monotonic_timestamp_ns(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            sample_time_pi.time_since_epoch())
            .count());
    output_builder.add_x(model_based_.xytheta()(0));
    output_builder.add_y(model_based_.xytheta()(1));
    output_builder.add_theta(model_based_.xytheta()(2));
    output_builder.add_zeroed(imu_watcher_.zeroer().Zeroed());
    output_builder.add_image_accepted_count(model_based_.total_accepted());
    const Eigen::Quaterniond &orientation = model_based_.orientation();
    Quaternion quaternion;
    quaternion.mutate_x(orientation.x());
    quaternion.mutate_y(orientation.y());
    quaternion.mutate_z(orientation.z());
    quaternion.mutate_w(orientation.w());
    output_builder.add_orientation(&quaternion);
    output_builder.add_led_outputs(led_outputs_offset);
    builder.CheckOk(builder.Send(output_builder.Finish()));
    last_output_send_ = event_loop_->monotonic_now();
  }
}

}  // namespace frc971::controls
