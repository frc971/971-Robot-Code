#ifndef FRC971_CONTROL_LOOPS_RUNGE_KUTTA_HELPERS_H_
#define FRC971_CONTROL_LOOPS_RUNGE_KUTTA_HELPERS_H_

#include "glog/logging.h"
#include <Eigen/Dense>

namespace frc971::control_loops {

// Returns a reasonable Runge Kutta initial step size. This is translated from
// scipy.
template <typename F, typename T>
double SelectRungeKuttaInitialStep(const F &fn, size_t t0, T y0, T f0,
                                   int error_estimator_order, double rtol,
                                   double atol) {
  constexpr int states = y0.rows();
  const Eigen::Matrix<double, states, 1> scale =
      atol + (y0.cwiseAbs().matrix() * rtol).array();
  const double sqrt_rows = std::sqrt(static_cast<double>(states));
  const double d0 = (y0.array() / scale.array()).matrix().norm() / sqrt_rows;
  const double d1 = (f0.array() / scale.array()).matrix().norm() / sqrt_rows;
  double h0;
  if (d0 < 1e-5 || d1 < 1e-5) {
    h0 = 1e-6;
  } else {
    h0 = 0.01 * d0 / d1;
  }

  const Eigen::Matrix<double, states, 1> y1 = y0 + h0 * f0;
  const Eigen::Matrix<double, states, 1> f1 = fn(t0 + h0, y1);
  const double d2 =
      ((f1 - f0).array() / scale.array()).matrix().norm() / sqrt_rows / h0;

  double h1;
  if (d1 <= 1e-15 && d2 <= 1e-15) {
    h1 = std::max(1e-6, h0 * 1e-3);
  } else {
    h1 = std::pow((0.01 / std::max(d1, d2)),
                  (1.0 / (error_estimator_order + 1.0)));
  }

  return std::min(100 * h0, h1);
}

// Performs a single step of Runge Kutta integration for the adaptive algorithm
// below. This is translated from scipy.
template <size_t N, size_t NStages, size_t Order, typename F>
std::tuple<Eigen::Matrix<double, N, 1>, Eigen::Matrix<double, N, 1>> RKStep(
    const F &fn, const double t, const Eigen::Matrix<double, N, 1> &y0,
    const Eigen::Matrix<double, N, 1> &f0, const double h,
    const Eigen::Matrix<double, NStages, Order> &A,
    const Eigen::Matrix<double, 1, NStages> &B,
    const Eigen::Matrix<double, 1, NStages> &C,
    Eigen::Matrix<double, NStages + 1, N> &K) {
  K.template block<N, 1>(0, 0) = f0;
  for (size_t s = 1; s < NStages; ++s) {
    Eigen::Matrix<double, N, 1> dy =
        K.block(0, 0, s, N).transpose() * A.block(s, 0, 1, s).transpose() * h;
    K.template block<1, N>(s, 0) = fn(t + C(0, s) * h, y0 + dy).transpose();
  }

  Eigen::Matrix<double, N, 1> y_new =
      y0 + h * (K.template block<NStages, N>(0, 0).transpose() * B.transpose());
  Eigen::Matrix<double, N, 1> f_new = fn(t + h, y_new);

  K.template block<1, N>(NStages, 0) = f_new.transpose();

  return std::make_tuple(y_new, f_new);
}

}  // namespace frc971::control_loops

#endif  // FRC971_CONTROL_LOOPS_RUNGE_KUTTA_HELPERS_H_
