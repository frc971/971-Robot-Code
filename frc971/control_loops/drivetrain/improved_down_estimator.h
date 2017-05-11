#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_IMPROVED_DOWN_ESTIMATOR_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_IMPROVED_DOWN_ESTIMATOR_H_

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "frc971/control_loops/runge_kutta.h"
#include "glog/logging.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

// Function to compute the quaternion average of a bunch of quaternions. Each
// column in the input matrix is a quaternion (optionally scaled by it's
// weight).
template <int SM>
Eigen::Matrix<double, 4, 1> QuaternionMean(
    Eigen::Matrix<double, 4, SM> input) {
  // Algorithm to compute the average of a bunch of quaternions:
  // http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf

  Eigen::Matrix<double, 4, 4> m = input * input.transpose();

  Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>> solver;
  solver.compute(m);

  Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>>::EigenvectorsType
      eigenvectors = solver.eigenvectors();
  Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>>::EigenvalueType eigenvalues =
      solver.eigenvalues();

  int max_index = 0;
  double max_eigenvalue = 0.0;
  for (int i = 0; i < 4; ++i) {
    const double eigenvalue = std::abs(eigenvalues(i, 0));
    if (eigenvalue > max_eigenvalue) {
      max_eigenvalue = eigenvalue;
      max_index = i;
    }
  }

  // Assume that there shouldn't be any imaginary components to the eigenvector.
  // I can't prove this is true, but everyone else seems to assume it...
  // TODO(james): Handle this more rigorously.
  for (int i = 0; i < 4; ++i) {
    CHECK_LT(eigenvectors(i, max_index).imag(), 1e-4)
        << eigenvectors(i, max_index);
  }
  return eigenvectors.col(max_index).real().normalized();
}

// Converts from a quaternion to a rotation vector, where the rotation vector's
// direction represents the axis to rotate around and its magnitude represents
// the number of radians to rotate.
Eigen::Matrix<double, 3, 1> ToRotationVectorFromQuaternion(
    const Eigen::Matrix<double, 4, 1> &X);

inline Eigen::Matrix<double, 3, 1> ToRotationVectorFromQuaternion(
    const Eigen::Quaternion<double> &X) {
  return ToRotationVectorFromQuaternion(X.coeffs());
};

// Converts from a rotation vector to a quaternion. If you supply max_angle_cap,
// then the rotation vector's magnitude will be clipped to be no more than
// max_angle_cap before being converted to a quaternion.
Eigen::Matrix<double, 4, 1> ToQuaternionFromRotationVector(
    const Eigen::Matrix<double, 3, 1> &X,
    const double max_angle_cap = std::numeric_limits<double>::infinity());

// Generates the sigma points to use in the UKF given the current estimate and
// covariance.
Eigen::Matrix<double, 4, 3 * 2 + 1> GenerateSigmaPoints(
    const Eigen::Quaternion<double> &mean,
    const Eigen::Matrix<double, 3, 3> &covariance);

// Computes the covariance of the noise given the mean and the transformed sigma
// points. The residual corresponds with the W' variable from the original
// paper.
Eigen::Matrix<double, 3, 3> ComputeQuaternionCovariance(
    const Eigen::Quaternion<double> &mean,
    const Eigen::Matrix<double, 4, 7> &points,
    Eigen::Matrix<double, 3, 7> *residual);

// This class provides a quaternion-based Kalman filter for estimating
// orientation using a 3-axis gyro and 3-axis accelerometer. It does leave open
// the option of overridding the system process model and the function used to
// calculate the expected measurement (which is relevant if, e.g., the IMU is
// not mounted horizontally in the robot).
class QuaternionUkf {
 public:
  // The state is just a quaternion representing the current robot orientaiton.
  // The zero/identity quaternion (1, 0, 0, 0) implies that the robot is
  // position flat on the ground with a heading of zero (which, in our normal
  // field coordinates, means pointed straight away from our driver's station
  // wall).
  constexpr static int kNumStates = 4;
  // Inputs to the system--we use the (x, y, z)  gyro measurements as the inputs
  // to the system.
  constexpr static int kNumInputs = 3;
  // Measurements to use for correcting the estimated system state. These
  // correspond to (x, y, z) measurements from the accelerometer.
  constexpr static int kNumMeasurements = 3;
  QuaternionUkf() {
    // TODO(james): Tune the process/measurement noises.
    R_.setIdentity();
    R_ /= 100.0;

    Q_.setIdentity();
    Q_ /= 10000.0;

    // Assume that the robot starts flat on the ground pointed straight forward.
    X_hat_ = Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0);

    // TODO(james): Determine an appropriate starting noise estimate. Probably
    // not too critical.
    P_.setIdentity();
    P_ /= 1000.0;
  }

  // Handles updating the state of the UKF, given the gyro and accelerometer
  // measurements.
  void Predict(const Eigen::Matrix<double, kNumInputs, 1> &U,
               const Eigen::Matrix<double, 3, 1> &measurement);

  // Returns the updated state for X after one time step, given the current
  // state and gyro measurements.
  virtual Eigen::Matrix<double, 4, 1> A(
      const Eigen::Matrix<double, 4, 1> &X,
      const Eigen::Matrix<double, kNumInputs, 1> &U) const = 0;

  // Returns the current expected accelerometer measurements given the current
  // state.
  virtual Eigen::Matrix<double, 3, 1> H(
      const Eigen::Matrix<double, 4, 1> &X) const = 0;

  // Returns the current estimate of the robot's orientation. Note that this
  // filter does not have anything other than the gyro with which to estimate
  // the robot's yaw heading, and so it may need to be corrected for by upstream
  // filters.
  const Eigen::Quaternion<double> &X_hat() const { return X_hat_; }

  Eigen::Matrix<double, 3, 1> Z_hat() const { return Z_hat_; };

 private:
  // Measurement Noise (Uncertainty)
  Eigen::Matrix<double, kNumInputs, kNumInputs> R_;
  // Model noise. Note that both this and P are 3 x 3 matrices, despite the
  // state having 4 dimensions.
  Eigen::Matrix<double, 3, 3> Q_;
  // Current estimate covariance.
  Eigen::Matrix<double, 3, 3> P_;

  // Current state estimate.
  Eigen::Quaternion<double> X_hat_;

  // Current expected accelerometer measurement.
  Eigen::Matrix<double, 3, 1> Z_hat_;
};

class DrivetrainUkf : public QuaternionUkf {
 public:
  constexpr static double kDt = 0.00505;

  // UKF for http://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf
  // Reference in case the link is dead:
  // Kraft, Edgar. "A quaternion-based unscented Kalman filter for orientation
  // tracking." In Proceedings of the Sixth International Conference of
  // Information Fusion, vol. 1, pp. 47-54. 2003.

  // A good reference for quaternions is available at
  // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/
  //
  // A good reference for angular velocity vectors with quaternions is at
  // http://www.euclideanspace.com/physics/kinematics/angularvelocity/

  // Creates a rotational velocity vector to be integrated.
  //
  // omega is the rotational velocity vector in body coordinates.
  // q is a matrix with the compononents of the quaternion in it.
  //
  // Returns dq / dt
  static Eigen::Vector4d QuaternionDerivative(Eigen::Vector3d omega,
                                              const Eigen::Vector4d &q_matrix) {
    Eigen::Quaternion<double> q(q_matrix);

    Eigen::Quaternion<double> omega_q;
    omega_q.w() = 0.0;
    omega_q.vec() = 0.5 * (q * omega);

    Eigen::Quaternion<double> deriv = omega_q * q;
    return deriv.coeffs();
  }

  // Moves the robot by the provided rotation vector (U).
  Eigen::Matrix<double, kNumStates, 1> A(
      const Eigen::Matrix<double, kNumStates, 1> &X,
      const Eigen::Matrix<double, kNumInputs, 1> &U) const override {
    return RungeKutta(
        std::bind(&QuaternionDerivative, U, std::placeholders::_1), X, kDt);
  }

  // Returns the expected accelerometer measurement (which is just going to be
  // 1g downwards).
  Eigen::Matrix<double, kNumMeasurements, 1> H(
      const Eigen::Matrix<double, kNumStates, 1> &X) const override {
    // TODO(austin): Figure out how to compute what the sensors *should* read.
    Eigen::Quaternion<double> Xquat(X);
    Eigen::Matrix<double, 3, 1> gprime =
        Xquat * Eigen::Matrix<double, 3, 1>(0.0, 0.0, -1.0);
    return gprime;
  }
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_IMPROVED_DOWN_ESTIMATOR_H_
