#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_SPLINE_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_SPLINE_H_

#include "Eigen/Dense"

#include "frc971/control_loops/binomial.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

// Class to hold a spline as a function of alpha.  Alpha can only range between
// 0.0 and 1.0.
template <int N>
class NSpline {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Computes the characteristic matrix of a given spline order. This is an
  // upper triangular matrix rather than lower because our splines are
  // represented as rows rather than columns.
  // Each row represents the impact of each point with increasing powers of
  // alpha. Row i, column j contains the effect point i has with the j'th power
  // of alpha.
  static ::Eigen::Matrix<double, N, N> SplineMatrix() {
    ::Eigen::Matrix<double, N, N> matrix =
        ::Eigen::Matrix<double, N, N>::Zero();

    for (int i = 0; i < N; ++i) {
      // Binomial(N - 1, i) * (1 - t) ^ (N - i - 1) * (t ^ i) * P[i]
      const double binomial = Binomial(N - 1, i);
      const int one_minus_t_power = N - i - 1;

      // Then iterate over the powers of t and add the pieces to the matrix.
      for (int j = i; j < N; ++j) {
        // j is the power of t we are placing in the matrix.
        // k is the power of t in the (1 - t) expression that we need to
        // evaluate.
        const int k = j - i;
        const double tscalar =
            binomial * Binomial(one_minus_t_power, k) * ::std::pow(-1.0, k);
        matrix(i, j) = tscalar;
      }
    }
    return matrix;
  }

  // Computes the matrix to multiply by [1, a, a^2, ...] to evaluate the spline.
  template <int D>
  static ::Eigen::Matrix<double, 2, N - D> SplinePolynomial(
      const ::Eigen::Matrix<double, 2, N> &control_points) {
    // We use rows for the spline, so the multiplication looks "backwards"
    ::Eigen::Matrix<double, 2, N> polynomial = control_points * SplineMatrix();

    // Now, compute the derivative requested.
    for (int i = D; i < N; ++i) {
      // Start with t^i, and multiply i, i-1, i-2 until you have done this
      // Derivative times.
      double scalar = 1.0;
      for (int j = i; j > i - D; --j) {
        scalar *= j;
      }
      polynomial.template block<2, 1>(0, i) =
          polynomial.template block<2, 1>(0, i) * scalar;
    }
    return polynomial.template block<2, N - D>(0, D);
  }

  // Computes an order M-1 polynomial matrix in alpha.  [1, alpha, alpha^2, ...]
  template <int M>
  static ::Eigen::Matrix<double, M, 1> AlphaPolynomial(const double alpha) {
    ::Eigen::Matrix<double, M, 1> polynomial =
        ::Eigen::Matrix<double, M, 1>::Zero();
    polynomial(0) = 1.0;
    for (int i = 1; i < M; ++i) {
      polynomial(i) = polynomial(i - 1) * alpha;
    }
    return polynomial;
  }

  // Constructs a spline.  control_points is a matrix of start, control1,
  // control2, ..., end.
  NSpline(::Eigen::Matrix<double, 2, N> control_points)
      : control_points_(control_points),
        spline_polynomial_(SplinePolynomial<0>(control_points_)),
        dspline_polynomial_(SplinePolynomial<1>(control_points_)),
        ddspline_polynomial_(SplinePolynomial<2>(control_points_)),
        dddspline_polynomial_(SplinePolynomial<3>(control_points_)) {}

  // Returns the xy coordiate of the spline for a given alpha.
  ::Eigen::Matrix<double, 2, 1> Point(double alpha) const {
    return spline_polynomial_ * AlphaPolynomial<N>(alpha);
  }

  // Returns the dspline/dalpha for a given alpha.
  ::Eigen::Matrix<double, 2, 1> DPoint(double alpha) const {
    return dspline_polynomial_ * AlphaPolynomial<N - 1>(alpha);
  }

  // Returns the d^2spline/dalpha^2 for a given alpha.
  ::Eigen::Matrix<double, 2, 1> DDPoint(double alpha) const {
    return ddspline_polynomial_ * AlphaPolynomial<N - 2>(alpha);
  }

  // Returns the d^3spline/dalpha^3 for a given alpha.
  ::Eigen::Matrix<double, 2, 1> DDDPoint(double alpha) const {
    return dddspline_polynomial_ * AlphaPolynomial<N - 3>(alpha);
  }

  // Returns theta for a given alpha.
  double Theta(double alpha) const {
    const ::Eigen::Matrix<double, 2, 1> dp = DPoint(alpha);
    return ::std::atan2(dp(1), dp(0));
  }

  // Returns dtheta/dalpha for a given alpha.
  double DTheta(double alpha) const {
    const ::Eigen::Matrix<double, N - 1, 1> alpha_polynomial =
        AlphaPolynomial<N - 1>(alpha);

    const ::Eigen::Matrix<double, 2, 1> dp =
        dspline_polynomial_ * alpha_polynomial;
    const ::Eigen::Matrix<double, 2, 1> ddp =
        ddspline_polynomial_ * alpha_polynomial.template block<N - 2, 1>(0, 0);
    const double dx = dp(0);
    const double dy = dp(1);

    const double ddx = ddp(0);
    const double ddy = ddp(1);

    return 1.0 / (::std::pow(dx, 2) + ::std::pow(dy, 2)) *
           (dx * ddy - dy * ddx);
  }

  // Returns d^2 theta/dalpha^2 for a given alpha.
  double DDTheta(double alpha) const {
    const ::Eigen::Matrix<double, N - 1, 1> alpha_polynomial =
        AlphaPolynomial<N - 1>(alpha);
    const ::Eigen::Matrix<double, 2, 1> dp =
        dspline_polynomial_ * alpha_polynomial;
    const ::Eigen::Matrix<double, 2, 1> ddp =
        ddspline_polynomial_ * alpha_polynomial.template block<N - 2, 1>(0, 0);
    const ::Eigen::Matrix<double, 2, 1> dddp =
        dddspline_polynomial_ * alpha_polynomial.template block<N - 3, 1>(0, 0);
    const double dx = dp(0);
    const double dy = dp(1);

    const double ddx = ddp(0);
    const double ddy = ddp(1);

    const double dddx = dddp(0);
    const double dddy = dddp(1);

    const double magdxy2 = ::std::pow(dx, 2) + ::std::pow(dy, 2);

    return -1.0 / (::std::pow(magdxy2, 2)) * (dx * ddy - dy * ddx) * 2.0 *
               (dy * ddy + dx * ddx) +
           1.0 / magdxy2 * (dx * dddy - dy * dddx);
  }

  const ::Eigen::Matrix<double, 2, N> &control_points() const {
    return control_points_;
  }

 private:
  const ::Eigen::Matrix<double, 2, N> control_points_;

  // Each of these polynomials gets multiplied by [x^(n-1), x^(n-2), ..., x, 1]
  // depending on the size of the polynomial.
  const ::Eigen::Matrix<double, 2, N> spline_polynomial_;
  const ::Eigen::Matrix<double, 2, N - 1> dspline_polynomial_;
  const ::Eigen::Matrix<double, 2, N - 2> ddspline_polynomial_;
  const ::Eigen::Matrix<double, 2, N - 3> dddspline_polynomial_;
};

typedef NSpline<6> Spline;

// Converts a 4 control point spline into
::Eigen::Matrix<double, 2, 6> Spline4To6(
    const ::Eigen::Matrix<double, 2, 4> &control_points);

template <int N>
::Eigen::Matrix<double, 2, N> TranslateSpline(
    const ::Eigen::Matrix<double, 2, N> &control_points,
    const ::Eigen::Matrix<double, 2, 1> translation) {
  ::Eigen::Matrix<double, 2, N> ans = control_points;
  for (size_t i = 0; i < N; ++i) {
    ans.template block<2, 1>(0, i) += translation;
  }
  return ans;
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_SPLINE_H_
