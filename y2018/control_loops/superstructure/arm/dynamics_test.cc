#include "y2018/control_loops/superstructure/arm/dynamics.h"

#include "gtest/gtest.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace arm {
namespace testing {

// Tests that zero inputs result in no acceleration and no motion.
// This isn't all that rigerous, but it's a good start.
TEST(DynamicsTest, Acceleration) {
  Dynamics dynamics;

  EXPECT_TRUE(dynamics
                  .Acceleration(::Eigen::Matrix<double, 4, 1>::Zero(),
                                ::Eigen::Matrix<double, 2, 1>::Zero())
                  .isApprox(::Eigen::Matrix<double, 4, 1>::Zero()));

  EXPECT_TRUE(dynamics
                  .UnboundedDiscreteDynamics(
                      ::Eigen::Matrix<double, 4, 1>::Zero(),
                      ::Eigen::Matrix<double, 2, 1>::Zero(), 0.1)
                  .isApprox(::Eigen::Matrix<double, 4, 1>::Zero()));

  const ::Eigen::Matrix<double, 4, 1> X =
      (::Eigen::Matrix<double, 4, 1>() << M_PI / 2.0, 0.0, 0.0, 0.0).finished();

  ::std::cout << dynamics.FF_U(X, ::Eigen::Matrix<double, 2, 1>::Zero(),
                               ::Eigen::Matrix<double, 2, 1>::Zero())
              << ::std::endl;

  ::std::cout << dynamics.UnboundedDiscreteDynamics(
                     X, dynamics.FF_U(X, ::Eigen::Matrix<double, 2, 1>::Zero(),
                                      ::Eigen::Matrix<double, 2, 1>::Zero()),
                     0.01)
              << ::std::endl;

  EXPECT_TRUE(dynamics
                  .UnboundedDiscreteDynamics(
                      X, dynamics.FF_U(X, ::Eigen::Matrix<double, 2, 1>::Zero(),
                                       ::Eigen::Matrix<double, 2, 1>::Zero()),
                      0.01)
                  .isApprox(X));
}

}  // namespace testing
}  // namespace arm
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
