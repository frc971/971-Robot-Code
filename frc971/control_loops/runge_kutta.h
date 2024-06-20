#ifndef FRC971_CONTROL_LOOPS_RUNGE_KUTTA_H_
#define FRC971_CONTROL_LOOPS_RUNGE_KUTTA_H_

#include "glog/logging.h"
#include <Eigen/Dense>

#include "frc971/control_loops/runge_kutta_helpers.h"

namespace frc971::control_loops {

// Implements Runge Kutta integration (4th order).  fn is the function to
// integrate.  It must take 1 argument of type T.  The integration starts at an
// initial value X, and integrates for dt.
template <typename F, typename T>
T RungeKutta(const F &fn, T X, double dt) {
  const double half_dt = dt * 0.5;
  T k1 = fn(X);
  T k2 = fn(X + half_dt * k1);
  T k3 = fn(X + half_dt * k2);
  T k4 = fn(X + dt * k3);
  return X + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

// Implements Runge Kutta integration (4th order) split up into steps steps.  fn
// is the function to integrate.  It must take 1 argument of type T.  The
// integration starts at an initial value X, and integrates for dt.
template <typename F, typename T>
T RungeKuttaSteps(const F &fn, T X, double dt, int steps) {
  dt = dt / steps;
  for (int i = 0; i < steps; ++i) {
    X = RungeKutta(fn, X, dt);
  }
  return X;
}

// Implements Runge Kutta integration (4th order).  This integrates dy/dt =
// fn(t, y).  It must have the call signature of fn(double t, T y).  The
// integration starts at an initial value y, and integrates for dt.
template <typename F, typename T>
T RungeKutta(const F &fn, T y, double t, double dt) {
  const double half_dt = dt * 0.5;
  T k1 = dt * fn(t, y);
  T k2 = dt * fn(t + half_dt, y + k1 / 2.0);
  T k3 = dt * fn(t + half_dt, y + k2 / 2.0);
  T k4 = dt * fn(t + dt, y + k3);

  return y + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
}

template <typename F, typename T>
T RungeKuttaSteps(const F &fn, T X, double t, double dt, int steps) {
  dt = dt / steps;
  for (int i = 0; i < steps; ++i) {
    X = RungeKutta(fn, X, t + dt * i, dt);
  }
  return X;
}

// Implements Runge Kutta integration (4th order).  fn is the function to
// integrate.  It must take 1 argument of type T.  The integration starts at an
// initial value X, and integrates for dt.
template <typename F, typename T, typename Tu>
T RungeKuttaU(const F &fn, T X, Tu U, double dt) {
  const double half_dt = dt * 0.5;
  T k1 = fn(X, U);
  T k2 = fn(X + half_dt * k1, U);
  T k3 = fn(X + half_dt * k2, U);
  T k4 = fn(X + dt * k3, U);
  return X + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

// Integrates f(t, y) from t0 to t0 + dt using an explicit Runge Kutta 5(4) to
// implement an adaptive step size.  Translated from Scipy.
//
// This uses the Dormand-Prince pair of formulas. The error is controlled
// assuming accuracy of the fourth-order method accuracy, but steps are taken
// using the fifth-order accurate formula (local extrapolation is done).  A
// quartic interpolation polynomial is used for the dense output.
//
// fn(t, y) is the function to integrate.  y0 is the initial y, t0 is the
// initial time, dt is the duration to integrate, rtol is the relative
// tolerance, and atol is the absolute tolerance.
template <typename F, typename T>
T AdaptiveRungeKutta(const F &fn, T y0, double t0, double dt,
                     double rtol = 1e-3, double atol = 1e-6) {
  // Multiply steps computed from asymptotic behaviour of errors by this.
  constexpr double SAFETY = 0.9;
  // Minimum allowed decrease in a step size.
  constexpr double MIN_FACTOR = 0.2;
  // Maximum allowed increase in a step size.
  constexpr double MAX_FACTOR = 10;

  // Final time
  const double t_bound = t0 + dt;

  constexpr int order = 5;
  constexpr int error_estimator_order = 4;
  constexpr int n_stages = 6;
  constexpr int states = y0.rows();
  const double sqrt_rows = std::sqrt(static_cast<double>(states));
  const Eigen::Matrix<double, 1, n_stages> C =
      (Eigen::Matrix<double, 1, n_stages>() << 0, 1.0 / 5.0, 3.0 / 10.0,
       4.0 / 5.0, 8.0 / 9.0, 1.0)
          .finished();

  const Eigen::Matrix<double, n_stages, order> A =
      (Eigen::Matrix<double, n_stages, order>() << 0.0, 0.0, 0.0, 0.0, 0.0,
       1.0 / 5.0, 0.0, 0.0, 0.0, 0.0, 3.0 / 40.0, 9.0 / 40.0, 0.0, 0.0, 0.0,
       44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0, 0.0, 0.0, 19372.0 / 6561.0,
       -25360.0 / 2187.0, 64448.0 / 6561.0, -212.0 / 729.0, 0.0,
       9017.0 / 3168.0, -355.0 / 33.0, 46732.0 / 5247.0, 49.0 / 176.0,
       -5103.0 / 18656.0)
          .finished();

  const Eigen::Matrix<double, 1, n_stages> B =
      (Eigen::Matrix<double, 1, n_stages>() << 35.0 / 384.0, 0.0,
       500.0 / 1113.0, 125.0 / 192.0, -2187.0 / 6784.0, 11.0 / 84.0)
          .finished();

  const Eigen::Matrix<double, 1, n_stages + 1> E =
      (Eigen::Matrix<double, 1, n_stages + 1>() << -71.0 / 57600.0, 0.0,
       71.0 / 16695.0, -71.0 / 1920.0, 17253.0 / 339200.0, -22.0 / 525.0,
       1.0 / 40.0)
          .finished();

  T f = fn(t0, y0);
  double h_abs = SelectRungeKuttaInitialStep(fn, t0, y0, f,
                                             error_estimator_order, rtol, atol);
  Eigen::Matrix<double, n_stages + 1, states> K;

  Eigen::Matrix<double, states, 1> y = y0;
  const double error_exponent = -1.0 / (error_estimator_order + 1.0);

  double t = t0;
  while (true) {
    if (t >= t_bound) {
      return y;
    }

    // Step
    double min_step =
        10 * (std::nextafter(t, std::numeric_limits<double>::infinity()) - t);

    // TODO(austin): max_step if we care.
    if (h_abs < min_step) {
      h_abs = min_step;
    }

    bool step_accepted = false;
    bool step_rejected = false;

    double t_new;
    Eigen::Matrix<double, states, 1> y_new;
    Eigen::Matrix<double, states, 1> f_new;
    while (!step_accepted) {
      // TODO(austin): Tell the user rather than just explode?
      CHECK_GE(h_abs, min_step);

      double h = h_abs;
      t_new = t + h;
      if (t_new >= t_bound) {
        t_new = t_bound;
      }
      h = t_new - t;
      h_abs = std::abs(h);

      std::tie(y_new, f_new) =
          RKStep<states, n_stages, order>(fn, t, y, f, h, A, B, C, K);

      const Eigen::Matrix<double, states, 1> scale =
          atol + y.array().abs().max(y_new.array().abs()) * rtol;

      double error_norm =
          (((K.transpose() * E.transpose()) * h).array() / scale.array())
              .matrix()
              .norm() /
          sqrt_rows;

      if (error_norm < 1) {
        double factor;
        if (error_norm == 0) {
          factor = MAX_FACTOR;
        } else {
          factor = std::min(MAX_FACTOR,
                            SAFETY * std::pow(error_norm, error_exponent));
        }

        if (step_rejected) {
          factor = std::min(1.0, factor);
        }

        h_abs *= factor;

        step_accepted = true;
      } else {
        h_abs *=
            std::max(MIN_FACTOR, SAFETY * std::pow(error_norm, error_exponent));
        step_rejected = true;
      }
    }

    t = t_new;
    y = y_new;
    f = f_new;
  }

  return y;
}

}  // namespace frc971::control_loops

#endif  // FRC971_CONTROL_LOOPS_RUNGE_KUTTA_H_
