#ifndef FRC971_MATH_EIGEN_MATCHERS_H_
#define FRC971_MATH_EIGEN_MATCHERS_H_

#include "gmock/gmock.h"

// This header provides useful gtest matchers for working with Eigen matrices.
namespace frc971::math {

MATCHER_P2(IsEigenMatrixNear, expected, threshold, "") {
  double difference_magnitude = (arg - expected).norm();
  if (difference_magnitude > threshold) {
    *result_listener << "matrix difference has a norm of "
                     << difference_magnitude << " versus a threshold of "
                     << threshold
                     << ". Matrix difference (actual - expected):\n"
                     << arg - expected;
    return false;
  }
  return true;
}

MATCHER_P2(AreEigenMatrixElementsNear, expected, element_thresholds, "") {
  auto difference = (arg - expected).array();
  auto out_of_tolerance = difference.cwiseAbs() > element_thresholds.array();
  if (out_of_tolerance.any()) {
    *result_listener << "matrix difference (actual - expected):\n"
                     << difference << "Out of tolerance elements:\n"
                     << out_of_tolerance;
    return false;
  }
  return true;
}
};      // namespace frc971::math
#endif  // FRC971_MATH_EIGEN_MATCHERS_H_
