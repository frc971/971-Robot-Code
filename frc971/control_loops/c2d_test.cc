#include "frc971/control_loops/c2d.h"

#include <functional>

#include "frc971/control_loops/runge_kutta.h"
#include "gtest/gtest.h"

namespace frc971 {
namespace controls {
namespace testing {

class C2DTest : public ::testing::Test {
 public:
  C2DTest() {
    // Create a trivial second-order system.
    A_continuous << 0, 1, 0, 0;
    B_continuous << 0, 1;
    Q_continuous << 1, 0, 0, 1;
  }

 protected:
  Eigen::Matrix<double, 2, 2> A_continuous;
  Eigen::Matrix<double, 2, 1> B_continuous;
  Eigen::Matrix<double, 2, 2> Q_continuous;
};

// Check that for a simple second-order system that we can easily analyze
// analytically, C2D creates valid A/B matrices.
TEST_F(C2DTest, DiscretizeAB) {
  Eigen::Matrix<double, 2, 1> X0;
  X0 << 1, 1;
  Eigen::Matrix<double, 1, 1> U;
  U << 1;
  Eigen::Matrix<double, 2, 2> A_d;
  Eigen::Matrix<double, 2, 1> B_d;

  C2D(A_continuous, B_continuous, ::std::chrono::seconds(1), &A_d, &B_d);
  Eigen::Matrix<double, 2, 1> X1_discrete = A_d * X0 + B_d * U;
  // We now have pos = vel = accel = 1, which should give us:
  Eigen::Matrix<double, 2, 1> X1_truth;
  X1_truth(1, 0) = X0(1, 0) + 1.0 * U(0, 0);
  X1_truth(0, 0) = X0(0, 0) + 1.0 * X0(1, 0) + 0.5 * U(0, 0);
  EXPECT_EQ(X1_truth, X1_discrete);
}

// Test that the discrete approximation of Q is roughly equal to
// integral from 0 to dt of e^(A tau) Q e^(A.T tau) dtau
TEST_F(C2DTest, DiscretizeQ) {
  Eigen::Matrix<double, 2, 2> Q_d;
  const auto dt = ::std::chrono::seconds(1);
  DiscretizeQ(Q_continuous, A_continuous, dt, &Q_d);
  // TODO(james): Using Runge Kutta for this is a bit silly as f is just a
  // function of t, not Q, but I don't want to rewrite any of our math
  // utilities.
  // Note that we are being very explicit about the types of everything in this
  // integration because otherwise it doesn't compile very well.
  Eigen::Matrix<double, 2, 2> Q_d_integrated = control_loops::RungeKutta<
      ::std::function<Eigen::Matrix<double, 2, 2>(
          const double, const Eigen::Matrix<double, 2, 2> &)>,
      Eigen::Matrix<double, 2, 2>>(
      [this](const double t, const Eigen::Matrix<double, 2, 2> &) {
        return Eigen::Matrix<double, 2, 2>(
            (A_continuous * t).exp() * Q_continuous *
            (A_continuous.transpose() * t).exp());
      },
      Eigen::Matrix<double, 2, 2>::Zero(), 0, 1.0);
  EXPECT_LT((Q_d_integrated - Q_d).norm(), 1e-10)
      << "Expected these to be nearly equal:\nQ_d:\n" << Q_d
      << "\nQ_d_integrated:\n" << Q_d_integrated;
}

// Tests that the "fast" discretization produces nearly identical results.
TEST_F(C2DTest, DiscretizeQAFast) {
  Eigen::Matrix<double, 2, 2> Q_d;
  Eigen::Matrix<double, 2, 2> Q_d_fast;
  Eigen::Matrix<double, 2, 2> A_d;
  Eigen::Matrix<double, 2, 2> A_d_fast;
  Eigen::Matrix<double, 2, 1> B_d;
  const auto dt = ::std::chrono::seconds(1);
  DiscretizeQ(Q_continuous, A_continuous, dt, &Q_d);
  C2D(A_continuous, B_continuous, dt, &A_d, &B_d);
  DiscretizeQAFast(Q_continuous, A_continuous, dt, &Q_d_fast, &A_d_fast);
  EXPECT_LT((Q_d - Q_d_fast).norm(), 1e-20);
  EXPECT_LT((A_d - A_d_fast).norm(), 1e-20);
}

}  // namespace testing
}  // namespace controls
}  // namespace frc971
