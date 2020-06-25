#ifndef AOS_CONTROLS_QUATERNION_UTILS_H_
#define AOS_CONTROLS_QUATERNION_UTILS_H_

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "glog/logging.h"

namespace aos {
namespace controls {

// Function to compute the quaternion average of a bunch of quaternions. Each
// column in the input matrix is a quaternion (optionally scaled by it's
// weight).
template <int SM>
inline Eigen::Matrix<double, 4, 1> QuaternionMean(
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
}

// Converts from a rotation vector to a quaternion. If you supply max_angle_cap,
// then the rotation vector's magnitude will be clipped to be no more than
// max_angle_cap before being converted to a quaternion.
Eigen::Matrix<double, 4, 1> ToQuaternionFromRotationVector(
    const Eigen::Matrix<double, 3, 1> &X,
    const double max_angle_cap = std::numeric_limits<double>::infinity());

}  // namespace controls
}  // namespace aos

#endif  // AOS_CONTROLS_QUATERNION_UTILS_H_
