#include "frc971/control_loops/drivetrain/improved_down_estimator.h"

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "aos/controls/quaternion_utils.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

void QuaternionUkf::Reset() {
  // The various noise matrices are tuned to provide values that seem
  // reasonable given our current setup (using the ADIS16470 for
  // measurements).
  R_.setIdentity();
  R_ /= std::pow(1000.0, 2);

  Q_.setIdentity();
  Q_ /= std::pow(2000.0 * 500.0, 2);

  // Assume that the robot starts flat on the ground pointed straight forward.
  X_hat_ = Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0);

  P_.setIdentity();
  P_ /= 1000.0;

  pos_vel_.setZero();
  for (auto &last_accel : last_accels_) {
    last_accel.setZero();
  }

  last_yaw_rates_.fill(0);
}

void QuaternionUkf::Predict(const Eigen::Matrix<double, 3, 1> &U,
                            const Eigen::Matrix<double, 3, 1> &measurement,
                            const aos::monotonic_clock::duration dt) {
  const Eigen::Matrix<double, 3, 1> calibrated_measurement =
      imu_transform_ * measurement;
  const Eigen::Matrix<double, 3, 1> calibrated_U = imu_transform_ * U;
  DoPredict(calibrated_U, calibrated_measurement, dt);
  IterationCleanup(calibrated_measurement, calibrated_U);
}

// States are X_hat_bar (position estimate) and P (Covariance)
void QuaternionUkf::DoPredict(const Eigen::Matrix<double, 3, 1> &U,
                              const Eigen::Matrix<double, 3, 1> &measurement,
                              const aos::monotonic_clock::duration dt) {
  // Compute the sigma points.
  // Our system is pretty linear. The traditional way of dealing with process
  // noise is to augment your state vector with the mean of the process noise,
  // and augment your covariance matrix with the covariance of your process
  // noise. Sigma points are then computed. These points are then propagated
  // through the model. This ends up effectively meaning that perturbations
  // from the unaugmented state with covariance P are propagated through the
  // model, and points which are at the mean but with perturbations to simulated
  // process noise are propagated through the system. The covariance is then
  // calculated from this set of points, and works out to have a covariance of
  // essentially P + Q.
  //
  // Since our noise is just additive, and quaternian rotation preserves
  // distance, we can add our noise first and it'll be a good representation of
  // our distance. This will reduce the number of math operations we need to
  // do. If we break this assumption in the future by adding a nonlinear model
  // somewhere in this system, we'll have to revisit this assumption.

  // Now, compute the actual sigma points using the columns of S as the
  // perturbation vectors. The last point is the original mean.
  const Eigen::Matrix<double, 4, 3 * 2 + 1> X =
      GenerateSigmaPoints(X_hat_, P_ + Q_);

  // Now, compute Y, the sigma points which have been propagated forwards by the
  // model.
  Eigen::Matrix<double, 4, 3 * 2 + 1> Y;
  for (int i = 0; i < Y.cols(); ++i) {
    // Y = Transformed sigma points
    Y.col(i) = A(X.col(i), U, dt);
  }

  // We now have the sigma points after the model update.
  // Compute the mean of the transformed sigma point
  X_hat_ = Eigen::Quaternion<double>(aos::controls::QuaternionMean(Y));

  // And the covariance.
  Eigen::Matrix<double, 3, 2 * 3 + 1> Wprime;
  Eigen::Matrix<double, 3, 3> P_prior =
      ComputeQuaternionCovariance(X_hat_, Y, &Wprime);

  // If the only obvious acceleration is that due to gravity, then accept the
  // measurement.
  const double kUseAccelThreshold = assume_perfect_gravity_ ? 1e-10 : 0.025;
  const double accel_norm = measurement.norm();
  if (std::abs(accel_norm - 1.0) > kUseAccelThreshold) {
    P_ = P_prior;
    consecutive_still_ = 0;
    return;
  }
  // Whenever we seem to have been still for a while, zero the integrated
  // velocity. Because we just use this for debugging, only set it once per time
  // duration when we are paused--this lets us observe how far things drift
  // while sitting still.
  if (consecutive_still_ == 1000) {
    pos_vel_.block<3, 1>(3, 0).setZero();
  }
  // Don't do accelerometer updates unless we have been roughly still for a
  // decent number of iterations.
  if (++consecutive_still_ < 50) {
    return;
  }

  // Update the gravity_magnitude_ using a semi-arbitrary time-constant that
  // seems to provide decent results.
  gravity_magnitude_ += 0.001 * (accel_norm - gravity_magnitude_);

  // TODO(austin): Maybe re-calculate the sigma points here before transforming
  // them?  Otherwise we can't cleanly decouple the model and measurement
  // updates.

  // Apply the measurement transform to all the sigma points to get a
  // representation of the distribution of the measurement.
  Eigen::Matrix<double, kNumMeasurements, 3 * 2 + 1> Z;
  Z_hat_.setZero();
  for (int i = 0; i < Z.cols(); ++i) {
    Z.col(i) = H(Y.col(i)) * accel_norm;

    // Compute the mean in addition.
    Z_hat_ += Z.col(i) / Z.cols();
  }

  // Now compute the measurement covariance.
  Eigen::Matrix<double, 3, 3> P_zz;
  P_zz.setZero();
  Eigen::Matrix<double, 3, 2 * 3 + 1> Zprime;
  for (int i = 0; i < 7; ++i) {
    // Compute the error vector for each sigma point.
    Eigen::Matrix<double, 3, 1> Zprimei = Z.col(i) - Z_hat_;

    // Now, compute the contribution of this sigma point to P_zz.
    P_zz += 1.0 / 12.0 * Zprimei * Zprimei.transpose();
    // Save the error for the cross-correlation matrix.
    Zprime.col(i) = Zprimei;
  }

  // Compute the measurement error and innovation uncertanty.
  const Eigen::Matrix<double, kNumMeasurements, kNumMeasurements> P_vv =
      P_zz + R_;

  // Now compute the cross correlation matrix P_xz.
  Eigen::Matrix<double, 3, 3> P_xz;
  P_xz.setZero();
  for (int i = 0; i < 7; ++i) {
    // Now, compute the contribution of this sigma point to P_prior.
    P_xz += 1.0 / 12.0 * Wprime.col(i) * Zprime.col(i).transpose();
  }

  // Compute the kalman gain.
  const Eigen::Matrix<double, 3, kNumMeasurements> K = P_xz * P_vv.inverse();

  // Update X_hat and the covariance P
  X_hat_ = X_hat_ * Eigen::Quaternion<double>(
                        aos::controls::ToQuaternionFromRotationVector(
                            K * (measurement - Z_hat_)));
  P_ = P_prior - K * P_vv * K.transpose();
}

void QuaternionUkf::IterationCleanup(const Eigen::Vector3d &accel,
                                     const Eigen::Vector3d &gyro) {
  // To make certain debugging simpler, resolve the ambiguity in the quaternion
  // space by making the w coefficient always be positive.
  if (X_hat_.coeffs().w() < 0) {
    X_hat_.coeffs() *= -1.0;
  }
  const Eigen::Vector3d robot_x_in_global_frame =
      X_hat() * Eigen::Vector3d::UnitX();
  const double yaw =
      std::atan2(robot_x_in_global_frame.y(), robot_x_in_global_frame.x());
  // The down estimator UKF does a poor job of estimating yaw, so back out and
  // remove the yaw estimate and let downstream estimators take care of it.
  X_hat_ = Eigen::AngleAxis<double>(-yaw, Eigen::Vector3d::UnitZ()) * X_hat();
  last_accels_[buffer_index_] =
      (X_hat_ * accel - Eigen::Vector3d::UnitZ() * gravity_magnitude_) *
      9.80665;
  last_yaw_rates_[buffer_index_] = gyro.z();
  buffer_index_ = (buffer_index_ + 1) % last_accels_.size();
}

Eigen::Matrix<double, 6, 1> QuaternionUkf::PosVelDerivative(
    const Eigen::Matrix<double, 6, 1> &pos_vel,
    const Eigen::Matrix<double, 3, 1> &accel) {
  Eigen::Matrix<double, 6, 1> derivative;
  derivative.block<3, 1>(0, 0) = pos_vel.block<3, 1>(3, 0);
  derivative.block<3, 1>(3, 0) = accel;
  return derivative;
}

void QuaternionUkf::UpdatePosition(aos::monotonic_clock::duration dt,
                                   const Eigen::Vector3d &accel) {
  const double dt_sec = aos::time::DurationInSeconds(dt);
  yaw_ += avg_recent_yaw_rates() * dt_sec;
  pos_vel_ = RungeKutta(
      std::bind(
          &QuaternionUkf::PosVelDerivative, this, std::placeholders::_1,
          Eigen::AngleAxis<double>(yaw_, Eigen::Vector3d::UnitZ()) * accel),
      pos_vel_, dt_sec);
}

Eigen::Matrix<double, 3, 3> ComputeQuaternionCovariance(
    const Eigen::Quaternion<double> &mean,
    const Eigen::Matrix<double, 4, 7> &points,
    Eigen::Matrix<double, 3, 7> *residual) {
  Eigen::Matrix<double, 3, 3> P_prior;
  P_prior.setZero();

  for (int i = 0; i < 7; ++i) {
    // Compute the error vector for each sigma point.
    Eigen::Matrix<double, 3, 1> Wprimei =
        aos::controls::ToRotationVectorFromQuaternion(
            Eigen::Quaternion<double>(mean).conjugate() *
            Eigen::Quaternion<double>(points.col(i)));
    // Now, compute the contribution of this sigma point to P_prior.
    P_prior += 1.0 / 6.0 * (Wprimei * Wprimei.transpose());
    // Save the error for the cross-correlation matrix.
    residual->col(i) = Wprimei;
  }
  return P_prior / 2.0;
}

Eigen::Matrix<double, 4, 3 * 2 + 1> GenerateSigmaPoints(
    const Eigen::Quaternion<double> &mean,
    const Eigen::Matrix<double, 3, 3> &covariance) {
  // Take the matrix square root.
  Eigen::Matrix<double, 3, 3> S = covariance.llt().matrixL();

  S *= std::sqrt(2.0 * 3.0);
  // TODO(austin): Make sure the sigma points aren't outside +- PI/2.0.
  // Otherwise they wrap on themselves and we get a mess.

  // Now, compute the actual sigma points using the columns of S as the
  // pertubation vectors. The last point is the original mean.
  Eigen::Matrix<double, 4, 3 * 2 + 1> X;
  for (int i = 0; i < 3; ++i) {
    Eigen::Quaternion<double> perturbation(
        aos::controls::ToQuaternionFromRotationVector(S.col(i), M_PI_2));

    X.col(i * 2) = (mean * perturbation).coeffs();
    X.col(i * 2 + 1) = (mean * perturbation.conjugate()).coeffs();
  }
  X.col(6) = mean.coeffs();
  return X;
}

void DrivetrainUkf::UpdateIntegratedPositions(
    aos::monotonic_clock::time_point now) {
  if (last_pos_vel_update_ != aos::monotonic_clock::min_time) {
    UpdatePosition(now - last_pos_vel_update_, avg_recent_accel());
  }
  last_pos_vel_update_ = now;
}

flatbuffers::Offset<DownEstimatorState> DrivetrainUkf::PopulateStatus(
    flatbuffers::FlatBufferBuilder *fbb, aos::monotonic_clock::time_point now) {
  // On everything but the first iteration, integrate the position/velocity
  // estimates.
  // TODO(james): Consider optionally disabling this to avoid excess CPU usage.
  UpdateIntegratedPositions(now);

  DownEstimatorState::Builder builder(*fbb);
  builder.add_quaternion_x(X_hat().x());
  builder.add_quaternion_y(X_hat().y());
  builder.add_quaternion_z(X_hat().z());
  builder.add_quaternion_w(X_hat().w());

  builder.add_yaw(yaw());

  // Calculate the current pitch numbers to provide a more human-readable
  // debugging output.
  {
    const Eigen::Vector3d robot_x_in_global_frame =
        X_hat() * Eigen::Vector3d::UnitX();
    const double xy_norm = robot_x_in_global_frame.block<2, 1>(0, 0).norm();
    const double pitch = std::atan2(-robot_x_in_global_frame.z(), xy_norm);

    builder.add_longitudinal_pitch(pitch);
  }
  {
    const Eigen::Vector3d robot_y_in_global_frame =
        X_hat() * Eigen::Vector3d::UnitY();
    const double xy_norm = robot_y_in_global_frame.block<2, 1>(0, 0).norm();
    builder.add_lateral_pitch(std::atan2(robot_y_in_global_frame.z(), xy_norm));
  }

  builder.add_position_x(pos_vel()(0, 0));
  builder.add_position_y(pos_vel()(1, 0));
  builder.add_position_z(pos_vel()(2, 0));
  builder.add_velocity_x(pos_vel()(3, 0));
  builder.add_velocity_y(pos_vel()(4, 0));
  builder.add_velocity_z(pos_vel()(5, 0));

  {
    const Eigen::Vector3d last_accel_avg = avg_recent_accel();
    builder.add_accel_x(last_accel_avg.x());
    builder.add_accel_y(last_accel_avg.y());
    builder.add_accel_z(last_accel_avg.z());
  }

  {
    const Eigen::Vector3d expected_accel = H(X_hat().coeffs());
    builder.add_expected_accel_x(expected_accel.x());
    builder.add_expected_accel_y(expected_accel.y());
    builder.add_expected_accel_z(expected_accel.z());
  }

  builder.add_gravity_magnitude(gravity_magnitude());

  builder.add_consecutive_still(consecutive_still());

  return builder.Finish();
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
