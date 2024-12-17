#ifndef FRC971_CONTROL_LOOPS_DARE_TEST_H_
#define FRC971_CONTROL_LOOPS_DARE_TEST_H_

#include "frc971/control_loops/dare.h"

#include "Eigen/Core"
#include "Eigen/Eigenvalues"
#include "absl/log/log.h"
#include "gtest/gtest.h"

namespace frc971::controls::testing {

void ExpectMatrixEqual(const Eigen::MatrixXd &lhs, const Eigen::MatrixXd &rhs,
                       double tolerance) {
  for (int row = 0; row < lhs.rows(); ++row) {
    for (int col = 0; col < lhs.cols(); ++col) {
      EXPECT_NEAR(lhs(row, col), rhs(row, col), tolerance);
      VLOG(1) << "row = " << row;
      VLOG(1) << "col = " << col;
    }
  }

  if (::testing::Test::HasFailure()) {
    VLOG(1) << "lhs =\n" << lhs << "\n";
    VLOG(1) << "rhs =\n" << rhs << "\n";
    VLOG(1) << "delta =\n" << Eigen::MatrixXd{lhs - rhs} << "\n";
  }
}

void ExpectPositiveSemidefinite(const Eigen::Ref<const Eigen::MatrixXd> &X) {
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigX{X,
                                                      Eigen::EigenvaluesOnly};
  for (int i = 0; i < X.rows(); ++i) {
    EXPECT_GE(eigX.eigenvalues()[i], 0.0);
  }
}

void ExpectDARESolution(const Eigen::Ref<const Eigen::MatrixXd> &A,
                        const Eigen::Ref<const Eigen::MatrixXd> &B,
                        const Eigen::Ref<const Eigen::MatrixXd> &Q,
                        const Eigen::Ref<const Eigen::MatrixXd> &R,
                        const Eigen::Ref<const Eigen::MatrixXd> &X) {
  // Check that X is the solution to the DARE
  // Y = AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0
  // clang-format off
  Eigen::MatrixXd Y =
      A.transpose() * X * A
      - X
      - (A.transpose() * X * B * (B.transpose() * X * B + R).inverse()
        * B.transpose() * X * A)
      + Q;
  // clang-format on
  ExpectMatrixEqual(Y, Eigen::MatrixXd::Zero(X.rows(), X.cols()), 1e-10);
}

void ExpectDARESolution(const Eigen::Ref<const Eigen::MatrixXd> &A,
                        const Eigen::Ref<const Eigen::MatrixXd> &B,
                        const Eigen::Ref<const Eigen::MatrixXd> &Q,
                        const Eigen::Ref<const Eigen::MatrixXd> &R,
                        const Eigen::Ref<const Eigen::MatrixXd> &N,
                        const Eigen::Ref<const Eigen::MatrixXd> &X) {
  // Check that X is the solution to the DARE
  // Y = AᵀXA − X − (AᵀXB + N)(BᵀXB + R)⁻¹(BᵀXA + Nᵀ) + Q = 0
  // clang-format off
  Eigen::MatrixXd Y =
      A.transpose() * X * A
      - X
      - ((A.transpose() * X * B + N) * (B.transpose() * X * B + R).inverse()
        * (B.transpose() * X * A + N.transpose()))
      + Q;
  // clang-format on
  ExpectMatrixEqual(Y, Eigen::MatrixXd::Zero(X.rows(), X.cols()), 1e-10);
}

TEST(DARETest, NonInvertibleA) {
  // Example 2 of "On the Numerical Solution of the Discrete-Time Algebraic
  // Riccati Equation"

  ::Eigen::Matrix<double, 4, 4> A{
      {0.5, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}, {0, 0, 0, 0}};
  ::Eigen::Matrix<double, 4, 1> B{{0}, {0}, {0}, {1}};
  ::Eigen::Matrix<double, 4, 4> Q{
      {1, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
  ::Eigen::Matrix<double, 1, 1> R{0.25};

  auto ret = frc971::controls::dare<double, 4, 1>(A, B, Q, R);
  EXPECT_TRUE(ret);
  auto X = ret.value();

  ExpectMatrixEqual(X, X.transpose(), 1e-10);
  ExpectPositiveSemidefinite(X);
  ExpectDARESolution(A, B, Q, R, X);
}

TEST(DARETest, InvertibleA) {
  ::Eigen::Matrix<double, 2, 2> A{{1, 1}, {0, 1}};
  ::Eigen::Matrix<double, 2, 1> B{{0}, {1}};
  ::Eigen::Matrix<double, 2, 2> Q{{1, 0}, {0, 0}};
  ::Eigen::Matrix<double, 1, 1> R{{0.3}};

  auto ret = frc971::controls::dare<double, 2, 1>(A, B, Q, R);
  EXPECT_TRUE(ret);
  auto X = ret.value();

  ExpectMatrixEqual(X, X.transpose(), 1e-10);
  ExpectPositiveSemidefinite(X);
  ExpectDARESolution(A, B, Q, R, X);
}

TEST(DARETest, FirstGeneralizedEigenvalueOfSTIsStable) {
  // The first generalized eigenvalue of (S, T) is stable

  ::Eigen::Matrix<double, 2, 2> A{{0, 1}, {0, 0}};
  ::Eigen::Matrix<double, 2, 1> B{{0}, {1}};
  ::Eigen::Matrix<double, 2, 2> Q{{1, 0}, {0, 1}};
  ::Eigen::Matrix<double, 1, 1> R{1};

  auto ret = frc971::controls::dare<double, 2, 1>(A, B, Q, R);
  EXPECT_TRUE(ret);
  auto X = ret.value();

  ExpectMatrixEqual(X, X.transpose(), 1e-10);
  ExpectPositiveSemidefinite(X);
  ExpectDARESolution(A, B, Q, R, X);
}

TEST(DARETest, IdentitySystem) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d Q{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d R{Eigen::Matrix2d::Identity()};

  auto ret = frc971::controls::dare<double, 2, 2>(A, B, Q, R);
  EXPECT_TRUE(ret);
  auto X = ret.value();

  ExpectMatrixEqual(X, X.transpose(), 1e-10);
  ExpectPositiveSemidefinite(X);
  ExpectDARESolution(A, B, Q, R, X);
}

TEST(DARETest, MoreInputsThanStates) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const ::Eigen::Matrix<double, 2, 3> B{{1.0, 0.0, 0.0}, {0.0, 0.5, 0.3}};
  const Eigen::Matrix2d Q{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};

  auto ret = frc971::controls::dare<double, 2, 3>(A, B, Q, R);
  EXPECT_TRUE(ret);
  auto X = ret.value();

  ExpectMatrixEqual(X, X.transpose(), 1e-10);
  ExpectPositiveSemidefinite(X);
  ExpectDARESolution(A, B, Q, R, X);
}

TEST(DARETest, QNotSymmetric) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d Q{{1.0, 1.0}, {0.0, 1.0}};
  const Eigen::Matrix2d R{Eigen::Matrix2d::Identity()};

  auto ret = frc971::controls::dare<double, 2, 2>(A, B, Q, R);
  EXPECT_FALSE(ret);
  EXPECT_EQ(ret.error(), frc971::controls::DareError::QNotSymmetric);
}

TEST(DARETest, QNotPositiveSemidefinite) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d Q{-Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d R{Eigen::Matrix2d::Identity()};

  auto ret = frc971::controls::dare<double, 2, 2>(A, B, Q, R);
  EXPECT_FALSE(ret);
  EXPECT_EQ(ret.error(), frc971::controls::DareError::QNotPositiveSemidefinite);
}

TEST(DARETest, RNotSymmetric) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d Q{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d R{{1.0, 1.0}, {0.0, 1.0}};

  auto ret = frc971::controls::dare<double, 2, 2>(A, B, Q, R);
  EXPECT_FALSE(ret);
  EXPECT_EQ(ret.error(), frc971::controls::DareError::RNotSymmetric);
}

TEST(DARETest, RNotPositiveDefinite) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d Q{Eigen::Matrix2d::Identity()};

  const Eigen::Matrix2d R1{Eigen::Matrix2d::Zero()};
  auto ret1 = frc971::controls::dare<double, 2, 2>(A, B, Q, R1);
  EXPECT_FALSE(ret1);
  EXPECT_EQ(ret1.error(), frc971::controls::DareError::RNotPositiveDefinite);

  const Eigen::Matrix2d R2{-Eigen::Matrix2d::Identity()};
  auto ret2 = frc971::controls::dare<double, 2, 2>(A, B, Q, R2);
  EXPECT_FALSE(ret2);
  EXPECT_EQ(ret2.error(), frc971::controls::DareError::RNotPositiveDefinite);
}

TEST(DARETest, ABNotStabilizable) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Zero()};
  const Eigen::Matrix2d Q{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d R{Eigen::Matrix2d::Identity()};

  auto ret = frc971::controls::dare<double, 2, 2>(A, B, Q, R);
  EXPECT_FALSE(ret);
  EXPECT_EQ(ret.error(), frc971::controls::DareError::ABNotStabilizable);
}

TEST(DARETest, ACNotDetectable) {
  const Eigen::Matrix2d A{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d Q{Eigen::Matrix2d::Zero()};
  const Eigen::Matrix2d R{Eigen::Matrix2d::Identity()};

  auto ret = frc971::controls::dare<double, 2, 2>(A, B, Q, R);
  EXPECT_FALSE(ret);
  EXPECT_EQ(ret.error(), frc971::controls::DareError::ACNotDetectable);
}

TEST(DARETest, QDecomposition) {
  // Ensures the decomposition of Q into CᵀC is correct

  const Eigen::Matrix2d A{{1.0, 0.0}, {0.0, 0.0}};
  const Eigen::Matrix2d B{Eigen::Matrix2d::Identity()};
  const Eigen::Matrix2d R{Eigen::Matrix2d::Identity()};

  // (A, C₁) should be detectable pair
  const Eigen::Matrix2d C_1{{0.0, 0.0}, {1.0, 0.0}};
  const Eigen::Matrix2d Q_1 = C_1.transpose() * C_1;
  auto ret1 = frc971::controls::dare<double, 2, 2>(A, B, Q_1, R);
  EXPECT_TRUE(ret1);

  // (A, C₂) shouldn't be detectable pair
  const Eigen::Matrix2d C_2 = C_1.transpose();
  const Eigen::Matrix2d Q_2 = C_2.transpose() * C_2;
  auto ret2 = frc971::controls::dare<double, 2, 2>(A, B, Q_2, R);
  EXPECT_FALSE(ret2);
  EXPECT_EQ(ret2.error(), frc971::controls::DareError::ACNotDetectable);
}

}  // namespace frc971::controls::testing

#endif  // FRC971_CONTROL_LOOPS_DARE_TEST_H_
