#include "frc971/control_loops/swerve/naive_estimator.h"

#include "aos/flatbuffers.h"
#include "frc971/math/flatbuffers_matrix.h"

namespace frc971::control_loops::swerve {
NaiveEstimator::NaiveEstimator(aos::EventLoop *event_loop,
                               const SwerveZeroing *zeroing_params,
                               const Parameters &params)
    : zeroing_{zeroing::ContinuousAbsoluteEncoderZeroingEstimator{
                   aos::UnpackFlatbuffer(zeroing_params->front_left())},
               zeroing::ContinuousAbsoluteEncoderZeroingEstimator{
                   aos::UnpackFlatbuffer(zeroing_params->front_right())},
               zeroing::ContinuousAbsoluteEncoderZeroingEstimator{
                   aos::UnpackFlatbuffer(zeroing_params->back_left())},
               zeroing::ContinuousAbsoluteEncoderZeroingEstimator{
                   aos::UnpackFlatbuffer(zeroing_params->back_right())}},
      state_(State::Zero()),
      params_(params),
      localizer_state_fetcher_(
          event_loop->MakeFetcher<LocalizerState>("/localizer")) {
  velocities_.fill(0);
  last_drive_positions_.fill(0);
}

NaiveEstimator::State NaiveEstimator::Update(
    aos::monotonic_clock::time_point now, const Position *position,
    const CanPosition *can_position, Scalar yaw_rate, Scalar accel_x,
    Scalar accel_y) {
  Eigen::Vector2d last_velocity{state_(States::kVx), state_(States::kVy)};

  bool first_iteration = false;
  // TODO(james): Standardize more things on  just using arrays of modules
  // rather than named modules....
  const std::array<aos::monotonic_clock::time_point, 4> drive_now{
      aos::monotonic_clock::time_point{std::chrono::nanoseconds(
          can_position->front_left()->translation()->timestamp())},
      aos::monotonic_clock::time_point{std::chrono::nanoseconds(
          can_position->front_right()->translation()->timestamp())},
      aos::monotonic_clock::time_point{std::chrono::nanoseconds(
          can_position->back_left()->translation()->timestamp())},
      aos::monotonic_clock::time_point{std::chrono::nanoseconds(
          can_position->back_right()->translation()->timestamp())}};
  if (!last_update_.has_value()) {
    first_iteration = true;
    last_update_ = now;
    last_drive_update_ = drive_now;
  }
  const Scalar dt = aos::time::DurationInSeconds(now - last_update_.value());

  const std::array<Scalar, 4> drive_positions{
      can_position->front_left()->translation()->position(),
      can_position->front_right()->translation()->position(),
      can_position->back_left()->translation()->position(),
      can_position->back_right()->translation()->position()};
  const std::array<const AbsolutePosition *, 4> rotation_positions{
      position->front_left()->rotation_position(),
      position->front_right()->rotation_position(),
      position->back_left()->rotation_position(),
      position->back_right()->rotation_position()};
  // Constants for a very simple IIR on the various velocities. Currently set to
  // do no filtering.
  constexpr Scalar kAlpha = 1.0;
  // To estimate overall robot vx/vy, we calculate the robot velocity implied by
  // each individual module and then average them all together. In cases with
  // low slip this is correct.
  Eigen::Vector2d accumulated_velocity = Eigen::Vector2d::Zero();
  for (size_t module = 0; module < 4; ++module) {
    const Scalar drive_dt = aos::time::DurationInSeconds(
        drive_now[module] - last_drive_update_[module]);
    // Detect situations where the drive motor readings for a given motor have
    // not changed since the last Update(). When this happens, we just re-use
    // the prior velocity estimate.
    if (drive_dt != 0.0) {
      const Scalar instantaneous_velocity =
          first_iteration
              ? 0.0
              : (drive_positions[module] - last_drive_positions_[module]) /
                    drive_dt;
      velocities_[module] = velocities_[module] * (1.0 - kAlpha) +
                            kAlpha * instantaneous_velocity;
    }
    const int thetas_idx = States::kThetas0 + States::kLength * module;
    const int omegas_idx = States::kOmegas0 + States::kLength * module;
    const int omegad_idx = States::kOmegad0 + States::kLength * module;
    {
      zeroing_[module].UpdateEstimate(*rotation_positions[module]);
      const Scalar yaw_encoder =
          rotation_positions[module]->encoder() + zeroing_[module].offset();
      const Scalar instantaneous_velocity =
          first_iteration ? 0.0 : (yaw_encoder - state_(thetas_idx)) / dt;
      state_(thetas_idx) = yaw_encoder;
      state_(omegas_idx) =
          state_(omegas_idx) * (1.0 - kAlpha) + kAlpha * instantaneous_velocity;
      state_(omegad_idx) = state_(omegad_idx) * (1.0 - kAlpha) +
                           kAlpha * instantaneous_velocity /
                               params_.modules[module].wheel_radius;
    }
    accumulated_velocity.x() +=
        std::cos(state_(thetas_idx) + state_(States::kTheta)) *
        velocities_[module];
    accumulated_velocity.y() +=
        std::sin(state_(thetas_idx) + state_(States::kTheta)) *
        velocities_[module];
  }
  accumulated_velocity /= 4.0;

  Eigen::Vector2d accel_based_velocity =
      last_velocity + dt * Eigen::Vector2d{accel_x, accel_y};
  // Take a weighted average for velocity between accelerometer and
  // encoder-based measurements.
  Eigen::Vector2d averaged_velocity =
      (1.0 - params_.accel_weight) * accumulated_velocity +
      params_.accel_weight * accel_based_velocity;

  state_(States::kOmega) = yaw_rate;
  state_(States::kVx) = averaged_velocity.x();
  state_(States::kVy) = averaged_velocity.y();
  if (!initial_theta_set_ || use_localizer_theta_) {
    localizer_state_fetcher_.Fetch();
    if (localizer_state_fetcher_.get() != nullptr) {
      state_(States::kTheta) = localizer_state_fetcher_->theta();
      initial_theta_set_ = true;
    }
  }

  state_(States::kTheta) += state_(States::kOmega) * dt;

  last_update_ = now;
  last_drive_update_ = drive_now;
  last_drive_positions_ = drive_positions;
  return state_;
}

void NaiveEstimator::PopulateStatus(NaiveEstimatorStatusStatic *fbs) const {
  CHECK(FromEigen(state_.cast<double>(), fbs->add_position_state()));
  auto estimator_states = fbs->add_estimator_states();
  CHECK(estimator_states->reserve(4));
  for (size_t module = 0; module < 4; ++module) {
    zeroing_[module].GetEstimatorState(estimator_states->emplace_back());
  }
  fbs->set_yaw(state_(States::kTheta));
  fbs->set_omega(state_(States::kOmega));
  fbs->set_vx(state_(States::kVx));
  fbs->set_vy(state_(States::kVy));

  CHECK(FromEigen(state_.template cast<double>(), fbs->add_x_hat()));
  CHECK(fbs->add_module_drive_velocities()->FromIterator(velocities_.begin(),
                                                         velocities_.end()));
}

}  // namespace frc971::control_loops::swerve
