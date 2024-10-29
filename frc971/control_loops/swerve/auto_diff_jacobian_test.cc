#include "frc971/control_loops/swerve/auto_diff_jacobian.h"

#include <functional>

#include "absl/log/log.h"
#include "gtest/gtest.h"

namespace frc971::control_loops::swerve::testing {
struct TestFunction {
  template <typename Scalar>
  Eigen::Matrix<Scalar, 3, 1> operator()(
      const Eigen::Map<const Eigen::Matrix<Scalar, 2, 1>> X) const {
    return Eigen::Matrix<double, 3, 2>{{1, 2}, {3, 4}, {5, 6}} * X;
  }
};

TEST(AutoDiffJacobianTest, EvaluatesJacobian) {
  EXPECT_EQ((AutoDiffJacobian<double, TestFunction, 2, 3>::Jacobian(
                TestFunction{}, Eigen::Vector2d::Zero())),
            (Eigen::Matrix<double, 3, 2>{{1, 2}, {3, 4}, {5, 6}}));
}

}  // namespace frc971::control_loops::swerve::testing
