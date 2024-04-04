#ifndef AOS_CONTROLS_QUATERNION_UTILS_H_
#define AOS_CONTROLS_QUATERNION_UTILS_H_

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "absl/log/check.h"
#include "absl/log/log.h"

namespace frc971::controls {

// Helper function to extract mean quaternion from A*A^T of quaternion list
// This allows us to support multiple formats of the input quaternion list
inline Eigen::Matrix<double, 4, 1> ExtractQuaternionMean(
    Eigen::Matrix<double, 4, 4> input) {
  // Algorithm to compute the average of a bunch of quaternions:
  //  http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf
  // See also:
  //  https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf

  Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>> solver;
  solver.compute(input);

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

// Function to compute the quaternion average of a bunch of quaternions. Each
// column in the 4xN input Matrix is a quaternion (optionally scaled by it's
// weight).
template <int N>
inline Eigen::Matrix<double, 4, 1> QuaternionMean(
    Eigen::Matrix<double, 4, N> quaternions) {
  Eigen::Matrix<double, 4, 4> m = quaternions * quaternions.transpose();

  return ExtractQuaternionMean(m);
}

// Function to compute the quaternion average of a bunch of quaternions.
// This allows for passing in a variable size list (vector) of quaternions

// For reference (since I've been bitten by it):
// Eigen::Quaternion stores and prints coefficients as [x, y, z, w]
// and initializes using a Vector4d in this order, BUT
// initializes with scalars as Eigen::Quaternion{w, x, y, z}
inline Eigen::Vector4d QuaternionMean(
    std::vector<Eigen::Vector4d> quaternion_list) {
  CHECK(quaternion_list.size() != 0)
      << "Must have at least one quaternion to compute an average!";

  Eigen::Matrix<double, 4, 4> m = Eigen::Matrix4d::Zero();
  for (Eigen::Vector4d quaternion : quaternion_list) {
    m += quaternion * quaternion.transpose();
  }

  return Eigen::Vector4d(ExtractQuaternionMean(m));
}

// Converts from a quaternion to a rotation vector, where the rotation vector's
// direction represents the axis to rotate around and its magnitude represents
// the number of radians to rotate.
Eigen::Matrix<double, 3, 1> ToRotationVectorFromQuaternion(
    const Eigen::Matrix<double, 4, 1> &X);

inline Eigen::Matrix<double, 3, 1> ToRotationVectorFromQuaternion(
    const Eigen::Quaternion<double> &X) {
  return ToRotationVectorFromQuaternion(X.coeffs());
}

// Converts from a rotation vector to a quaternion. If you supply max_angle_cap,
// then the rotation vector's magnitude will be clipped to be no more than
// max_angle_cap before being converted to a quaternion.
Eigen::Matrix<double, 4, 1> ToQuaternionFromRotationVector(
    const Eigen::Matrix<double, 3, 1> &X,
    const double max_angle_cap = std::numeric_limits<double>::infinity());

// Creates a rotational velocity vector to be integrated.
//
// omega is the rotational velocity vector in body coordinates.
// q is a matrix with the compononents of the quaternion in it.
//
// Returns dq / dt
inline Eigen::Vector4d QuaternionDerivative(Eigen::Vector3d omega,
                                            const Eigen::Vector4d &q_matrix) {
  // See https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/ for
  // another resource on quaternion integration and derivatives.
  Eigen::Quaternion<double> q(q_matrix);

  Eigen::Quaternion<double> omega_q;
  omega_q.w() = 0.0;
  omega_q.vec() = 0.5 * omega;

  Eigen::Quaternion<double> deriv = q * omega_q;
  return deriv.coeffs();
}

// d QuaternionDerivative / d omega
Eigen::Matrix<double, 4, 3> QuaternionDerivativeDerivitive(
    const Eigen::Vector4d &q_matrix);
inline Eigen::Matrix<double, 4, 3> QuaternionDerivativeDerivitive(
    const Eigen::Quaternion<double> &q) {
  return QuaternionDerivativeDerivitive(q.coeffs());
}

}  // namespace frc971::controls

#endif  // AOS_CONTROLS_QUATERNION_UTILS_H_
