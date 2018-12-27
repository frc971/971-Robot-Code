#include "frc971/control_loops/runge_kutta.h"

#include "gtest/gtest.h"

namespace frc971 {
namespace control_loops {
namespace testing {

// Tests that integrating dx/dt = e^x works.
TEST(RungeKuttaTest, Exponential) {
  ::Eigen::Matrix<double, 1, 1> y0;
  y0(0, 0) = 0.0;

  ::Eigen::Matrix<double, 1, 1> y1 = RungeKutta(
      [](::Eigen::Matrix<double, 1, 1> x) {
        ::Eigen::Matrix<double, 1, 1> y;
        y(0, 0) = ::std::exp(x(0, 0));
        return y;
      },
      y0, 0.1);
  EXPECT_NEAR(y1(0, 0), ::std::exp(0.1) - ::std::exp(0), 1e-3);
}

// Tests that integrating dx/dt = e^x works when we provide a U.
TEST(RungeKuttaTest, ExponentialWithU) {
  ::Eigen::Matrix<double, 1, 1> y0;
  y0(0, 0) = 0.0;

  ::Eigen::Matrix<double, 1, 1> y1 = RungeKuttaU(
      [](::Eigen::Matrix<double, 1, 1> x, ::Eigen::Matrix<double, 1, 1> u) {
        ::Eigen::Matrix<double, 1, 1> y;
        y(0, 0) = ::std::exp(u(0, 0) * x(0, 0));
        return y;
      },
      y0, (::Eigen::Matrix<double, 1, 1>() << 1.0).finished(), 0.1);
  EXPECT_NEAR(y1(0, 0), ::std::exp(0.1) - ::std::exp(0), 1e-3);
}

}  // namespace testing
}  // namespace control_loops
}  // namespace frc971
