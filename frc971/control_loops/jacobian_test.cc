#include "frc971/control_loops/jacobian.h"

#include "gtest/gtest.h"

namespace frc971 {
namespace control_loops {
namespace testing {

::Eigen::Matrix<double, 4, 4> A = (::Eigen::Matrix<double, 4, 4>() << 1, 2, 4,
                                   1, 5, 2, 3, 4, 5, 1, 3, 2, 1, 1, 3,
                                   7).finished();

::Eigen::Matrix<double, 4, 2> B =
    (::Eigen::Matrix<double, 4, 2>() << 1, 1, 2, 1, 3, 2, 3, 7).finished();

// Function to recover A and B from.
::Eigen::Matrix<double, 4, 1> AxBufn(const ::Eigen::Matrix<double, 4, 1> &X,
                                     const ::Eigen::Matrix<double, 2, 1> &U) {
  return A * X + B * U;
}

// Test that we can recover A from AxBufn pretty accurately.
TEST(RungeKuttaTest, Ax) {
  ::Eigen::Matrix<double, 4, 4> NewA =
      NumericalJacobianX<4, 2>(AxBufn, ::Eigen::Matrix<double, 4, 1>::Zero(),
                               ::Eigen::Matrix<double, 2, 1>::Zero());
  EXPECT_TRUE(NewA.isApprox(A));
}

// Test that we can recover B from AxBufn pretty accurately.
TEST(RungeKuttaTest, Bu) {
  ::Eigen::Matrix<double, 4, 2> NewB =
      NumericalJacobianU<4, 2>(AxBufn, ::Eigen::Matrix<double, 4, 1>::Zero(),
                               ::Eigen::Matrix<double, 2, 1>::Zero());
  EXPECT_TRUE(NewB.isApprox(B));
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
