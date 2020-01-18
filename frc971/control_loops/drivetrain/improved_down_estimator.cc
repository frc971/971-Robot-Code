#include "frc971/control_loops/drivetrain/improved_down_estimator.h"

#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

Eigen::Matrix<double, 4, 1> ToQuaternionFromRotationVector(
    const Eigen::Matrix<double, 3, 1> &X, const double max_angle_cap) {
  const double unclipped_angle = X.norm();
  const double angle_scalar =
      (unclipped_angle > max_angle_cap) ? max_angle_cap / unclipped_angle : 1.0;
  const double angle = unclipped_angle * angle_scalar;
  const double half_angle = angle * 0.5;

  const double half_angle_squared = half_angle * half_angle;

  // sin(x)/x = 1
  double sinx_x = 1.0;

  // - x^2/3!
  double value = half_angle_squared / 6.0;
  sinx_x -= value;

  // + x^4/5!
  value = value * half_angle_squared / 20.0;
  sinx_x += value;

  // - x^6/7!
  value = value * half_angle_squared / (6.0 * 7.0);
  sinx_x -= value;

  // + x^8/9!
  value = value * half_angle_squared / (8.0 * 9.0);
  sinx_x += value;

  // - x^10/11!
  value = value * half_angle_squared / (10.0 * 11.0);
  sinx_x -= value;

  // + x^12/13!
  value = value * half_angle_squared / (12.0 * 13.0);
  sinx_x += value;

  // - x^14/15!
  value = value * half_angle_squared / (14.0 * 15.0);
  sinx_x -= value;

  // + x^16/17!
  value = value * half_angle_squared / (16.0 * 17.0);
  sinx_x += value;

  // To plot the residual in matplotlib, run:
  // import numpy
  // import scipy
  // from matplotlib import pyplot
  // x = numpy.arange(-numpy.pi, numpy.pi, 0.01)
  // pyplot.plot(x, 1 - x**2 / scipy.misc.factorial(3) +
  //                   x**4 / scipy.misc.factorial(5) -
  //                   x**6 / scipy.misc.factorial(7) +
  //                   x**8 / scipy.misc.factorial(9) -
  //                   x ** 10 / scipy.misc.factorial(11) +
  //                   x ** 12 / scipy.misc.factorial(13) -
  //                   x ** 14 / scipy.misc.factorial(15) +
  //                   x ** 16 / scipy.misc.factorial(17) -
  //                   numpy.sin(x) / x)

  const double scalar = sinx_x * 0.5;

  Eigen::Matrix<double, 4, 1> result;
  result.block<3, 1>(0, 0) = X * scalar * angle_scalar;
  result(3, 0) = std::cos(half_angle);
  return result;
}

inline Eigen::Matrix<double, 4, 1> MaybeFlipX(
    const Eigen::Matrix<double, 4, 1> &X) {
  if (X(3, 0) < 0.0) {
    return -X;
  } else {
    return X;
  }
}

Eigen::Matrix<double, 3, 1> ToRotationVectorFromQuaternion(
    const Eigen::Matrix<double, 4, 1> &X) {
  // TODO(austin): Verify we still need it.
  const Eigen::Matrix<double, 4, 1> corrected_X = MaybeFlipX(X);
  const double half_angle =
      std::atan2(corrected_X.block<3, 1>(0, 0).norm(), corrected_X(3, 0));

  const double half_angle_squared = half_angle * half_angle;

  // TODO(austin): We are doing a division at the end of this. Do the taylor
  // series expansion of x/sin(x) instead to avoid this.

  // sin(x)/x = 1
  double sinx_x = 1.0;

  // - x^2/3!
  double value = half_angle_squared / 6.0;
  sinx_x -= value;

  // + x^4/5!
  value = value * half_angle_squared / 20.0;
  sinx_x += value;

  // - x^6/7!
  value = value * half_angle_squared / (6.0 * 7.0);
  sinx_x -= value;

  // + x^8/9!
  value = value * half_angle_squared / (8.0 * 9.0);
  sinx_x += value;

  // - x^10/11!
  value = value * half_angle_squared / (10.0 * 11.0);
  sinx_x -= value;

  // + x^12/13!
  value = value * half_angle_squared / (12.0 * 13.0);
  sinx_x += value;

  // - x^14/15!
  value = value * half_angle_squared / (14.0 * 15.0);
  sinx_x -= value;

  // + x^16/17!
  value = value * half_angle_squared / (16.0 * 17.0);
  sinx_x += value;

  const double scalar = 2.0 / sinx_x;

  return corrected_X.block<3, 1>(0, 0) * scalar;
}

// States are X_hat_bar (position estimate) and P (Covariance)

void QuaternionUkf::Predict(const Eigen::Matrix<double, 3, 1> &U,
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
  // pertubation vectors. The last point is the original mean.
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
  X_hat_ = Eigen::Quaternion<double>(QuaternionMean(Y));

  // And the covariance.
  Eigen::Matrix<double, 3, 2 * 3 + 1> Wprime;
  Eigen::Matrix<double, 3, 3> P_prior =
      ComputeQuaternionCovariance(X_hat_, Y, &Wprime);

  // If the only obvious acceleration is that due to gravity, then accept the
  // measurement.
  // TODO(james): Calibrate this on a real robot. This may require some sort of
  // calibration routine.
  constexpr double kUseAccelThreshold = 0.02;
  if (std::abs(measurement.squaredNorm() - 1.0) > kUseAccelThreshold) {
    P_ = P_prior;
    return;
  }

  // TODO(austin): Maybe re-calculate the sigma points here before transforming
  // them?  Otherwise we can't cleanly decouple the model and measurement
  // updates.

  // Apply the measurement transform to all the sigma points to get a
  // representation of the distribution of the measurement.
  Eigen::Matrix<double, kNumMeasurements, 3 * 2 + 1> Z;
  Z_hat_.setZero();
  for (int i = 0; i < Z.cols(); ++i) {
    Z.col(i) = H(Y.col(i));

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
  const Eigen::Matrix<double, 3, kNumMeasurements> K =
      P_xz * P_vv.inverse();

  // Update X_hat and the covariance P
  X_hat_ = X_hat_ * Eigen::Quaternion<double>(ToQuaternionFromRotationVector(
                        K * (measurement - Z_hat_)));
  P_ = P_prior - K * P_vv * K.transpose();
}

Eigen::Matrix<double, 3, 3> ComputeQuaternionCovariance(
    const Eigen::Quaternion<double> &mean,
    const Eigen::Matrix<double, 4, 7> &points,
    Eigen::Matrix<double, 3, 7> *residual) {
  Eigen::Matrix<double, 3, 3> P_prior;
  P_prior.setZero();

  for (int i = 0; i < 7; ++i) {
    // Compute the error vector for each sigma point.
    Eigen::Matrix<double, 3, 1> Wprimei = ToRotationVectorFromQuaternion(
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
        ToQuaternionFromRotationVector(S.col(i), M_PI_2));

    X.col(i * 2) = (mean * perturbation).coeffs();
    X.col(i * 2 + 1) = (mean * perturbation.conjugate()).coeffs();
  }
  X.col(6) = mean.coeffs();
  return X;
}

flatbuffers::Offset<DownEstimatorState> DrivetrainUkf::PopulateStatus(
    flatbuffers::FlatBufferBuilder *fbb) const {
  DownEstimatorState::Builder builder(*fbb);
  builder.add_quaternion_x(X_hat().x());
  builder.add_quaternion_y(X_hat().y());
  builder.add_quaternion_z(X_hat().z());
  builder.add_quaternion_w(X_hat().w());

  {
    // Note that this algorithm is not very numerically stable near pitches of
    // +/- pi / 2.
    const Eigen::Vector3d robot_x_in_global_frame =
        X_hat() * Eigen::Vector3d::UnitX();
    builder.add_yaw(
        std::atan2(robot_x_in_global_frame.y(), robot_x_in_global_frame.x()));
    const double xy_norm = robot_x_in_global_frame.block<2, 1>(0, 0).norm();
    builder.add_longitudinal_pitch(
        std::atan2(-robot_x_in_global_frame.z(), xy_norm));
  }
  {
    const Eigen::Vector3d robot_y_in_global_frame =
        X_hat() * Eigen::Vector3d::UnitY();
    const double xy_norm = robot_y_in_global_frame.block<2, 1>(0, 0).norm();
    builder.add_lateral_pitch(
        std::atan2(robot_y_in_global_frame.z(), xy_norm));
  }

  return builder.Finish();
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
