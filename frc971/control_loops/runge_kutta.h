#ifndef FRC971_CONTROL_LOOPS_RUNGE_KUTTA_H_
#define FRC971_CONTROL_LOOPS_RUNGE_KUTTA_H_

#include <Eigen/Dense>

namespace frc971 {
namespace control_loops {

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

// Implements Runge Kutta integration (4th order).  This integrates dy/dt = fn(t,
// y).  It must have the call signature of fn(double t, T y).  The
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

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_RUNGE_KUTTA_H_
