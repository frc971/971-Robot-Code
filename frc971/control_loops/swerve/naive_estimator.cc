#include "frc971/control_loops/swerve/naive_estimator.h"

#include "aos/flatbuffers.h"
#include "frc971/math/flatbuffers_matrix.h"

namespace frc971::control_loops::swerve {
NaiveEstimator::NaiveEstimator(const SwerveZeroing *zeroing_params,
                               const Parameters &)
    : zeroing_{zeroing::ContinuousAbsoluteEncoderZeroingEstimator{
                   aos::UnpackFlatbuffer(zeroing_params->front_left())},
               zeroing::ContinuousAbsoluteEncoderZeroingEstimator{
                   aos::UnpackFlatbuffer(zeroing_params->front_right())},
               zeroing::ContinuousAbsoluteEncoderZeroingEstimator{
                   aos::UnpackFlatbuffer(zeroing_params->back_left())},
               zeroing::ContinuousAbsoluteEncoderZeroingEstimator{
                   aos::UnpackFlatbuffer(zeroing_params->back_right())}},
      state_(State::Zero()) {
  velocities_.fill(0);
  last_drive_positions_.fill(0);
}

NaiveEstimator::State NaiveEstimator::Update(
    aos::monotonic_clock::time_point now, const Position *position,
    const CanPosition *can_position, Scalar yaw_rate) {
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
  // To estimate voerall robot vx/vy, we calculate the robot velocity implied by
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
    const int theta_idx = States::kThetas0 + 2 * module;
    const int omega_idx = States::kOmegas0 + 2 * module;
    {
      zeroing_[module].UpdateEstimate(*rotation_positions[module]);
      const Scalar yaw_encoder =
          rotation_positions[module]->encoder() + zeroing_[module].offset();
      const Scalar instantaneous_velocity =
          first_iteration ? 0.0 : (yaw_encoder - state_(theta_idx)) / dt;
      state_(theta_idx) = yaw_encoder;
      state_(omega_idx) =
          state_(omega_idx) * (1.0 - kAlpha) + kAlpha * instantaneous_velocity;
    }
    accumulated_velocity.x() +=
        std::cos(state_(theta_idx) + state_(States::kTheta)) *
        velocities_[module];
    accumulated_velocity.y() +=
        std::sin(state_(theta_idx) + state_(States::kTheta)) *
        velocities_[module];
  }
  accumulated_velocity /= 4.0;

  state_(States::kOmega) = yaw_rate;
  state_(States::kTheta) += yaw_rate * dt;
  state_(States::kVx) = accumulated_velocity.x();
  state_(States::kVy) = accumulated_velocity.y();
  last_update_ = now;
  last_drive_update_ = drive_now;
  last_drive_positions_ = drive_positions;
  return state_;
}

void NaiveEstimator::PopulateStatus(NaiveEstimatorStatusStatic *fbs) const {
  CHECK(FromEigen(state_.cast<double>(), fbs->add_velocity_state()));
  auto estimator_states = fbs->add_estimator_states();
  CHECK(estimator_states->reserve(4));
  for (size_t module = 0; module < 4; ++module) {
    zeroing_[module].GetEstimatorState(estimator_states->emplace_back());
  }
  fbs->set_omega(state_(States::kOmega));
  fbs->set_yaw(state_(States::kTheta));
  fbs->set_vx(state_(States::kVx));
  fbs->set_vy(state_(States::kVy));
  CHECK(FromEigen(state_.template cast<double>(), fbs->add_x_hat()));
  CHECK(fbs->add_module_drive_velocities()->FromIterator(velocities_.begin(),
                                                         velocities_.end()));
}
}  // namespace frc971::control_loops::swerve
