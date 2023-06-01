#include "frc971/solvers/sparse_convex.h"

#include "gtest/gtest.h"

namespace frc971 {
namespace solvers {
namespace testing {

const Eigen::IOFormat kHeavyFormat(Eigen::StreamPrecision, 0, ", ",
                                   ",\n                        "
                                   "                                     ",
                                   "[", "]", "[", "]");

class SimpleQP : public SparseConvexProblem {
 public:
  // QP of the for 0.5 * X^t Q_ X + p.T * X
  SimpleQP(Eigen::Matrix<double, 2, 2> Q, Eigen::Matrix<double, 2, 1> p,
           double x0_max, double x0_min, double x1_max, double x1_min)
      : SparseConvexProblem(2, 4, 1), Q_(Q), p_(p) {
    C_ << 1, 0, -1, 0, 0, 1, 0, -1;
    c_ << x0_max, -x0_min, x1_max, -x1_min;
  }

  double f0(Eigen::Ref<const Eigen::VectorXd> X) const override {
    return 0.5 * (X.transpose() * Q_ * X)(0, 0);
  }

  Eigen::SparseMatrix<double> df0(
      Eigen::Ref<const Eigen::VectorXd> X) const override {
    return (Q_ * X + p_).sparseView();
  }

  Eigen::SparseMatrix<double> ddf0(
      Eigen::Ref<const Eigen::VectorXd> /*X*/) const override {
    return Q_.sparseView();
  }

  // Returns the constraints f(X) < 0, and their derivitive.
  Eigen::VectorXd f(
      Eigen::Ref<const Eigen::VectorXd> X) const override {
    return C_ * X - c_;
  }
  Eigen::SparseMatrix<double> df(
      Eigen::Ref<const Eigen::VectorXd> /*X*/) const override {
    return C_.sparseView();
  }

  // Returns the equality constraints of the form A x = b
  Eigen::SparseMatrix<double> A() const override {
    return Eigen::Matrix<double, 1, 2>(1, -1).sparseView();
  }
  Eigen::VectorXd b() const override {
    return Eigen::Matrix<double, 1, 1>(0);
  }

 private:
  Eigen::Matrix<double, 2, 2> Q_;
  Eigen::Matrix<double, 2, 1> p_;

  Eigen::Matrix<double, 4, 2> C_;
  Eigen::Matrix<double, 4, 1> c_;
};

// Test a constrained quadratic problem where the constraints aren't active.
TEST(SolverTest, SimpleQP) {
  Eigen::Matrix<double, 2, 2> Q = Eigen::DiagonalMatrix<double, 2>(1.0, 1.0);
  Eigen::Matrix<double, 2, 1> p(-4, -6);

  SimpleQP qp(Q, p, 6, -1, 6, -1);
  SparseSolver s;
  Eigen::Vector2d result = s.Solve(qp, Eigen::Matrix<double, 2, 1>(0, 0));
  LOG(INFO) << "Result is " << std::setprecision(12)
            << result.transpose().format(kHeavyFormat);
  EXPECT_NEAR((result - Eigen::Vector2d(5.0, 5.0)).norm(), 0.0, 1e-6);
}

// Test a constrained quadratic problem where the constraints are active.
TEST(SolverTest, Constrained) {
  Eigen::Matrix<double, 2, 2> Q = Eigen::DiagonalMatrix<double, 2>(1.0, 2.0);
  Eigen::Matrix<double, 2, 1> p(-5, -10);

  SimpleQP qp(Q, p, 4, -1, 5, -1);
  SparseSolver s;
  Eigen::Vector2d result = s.Solve(qp, Eigen::Matrix<double, 2, 1>(3, 4));
  LOG(INFO) << "Result is " << std::setprecision(12)
            << result.transpose().format(kHeavyFormat);
  EXPECT_NEAR((result - Eigen::Vector2d(4.0, 4.0)).norm(), 0.0, 1e-6);
}

// Test a constrained quadratic problem where the constraints are active and the
// initial value is the solution.
TEST(SolverTest, ConstrainedFromSolution) {
  Eigen::Matrix<double, 2, 2> Q = Eigen::DiagonalMatrix<double, 2>(1.0, 2.0);
  Eigen::Matrix<double, 2, 1> p(-5, -10);

  SimpleQP qp(Q, p, 4, -1, 5, -1);
  SparseSolver s;
  Eigen::Vector2d result = s.Solve(qp, Eigen::Matrix<double, 2, 1>(4, 4));
  LOG(INFO) << "Result is " << std::setprecision(12)
            << result.transpose().format(kHeavyFormat);
  EXPECT_NEAR((result - Eigen::Vector2d(4.0, 4.0)).norm(), 0.0, 1e-6);
}

}  // namespace testing
}  // namespace solvers
}  // namespace frc971
