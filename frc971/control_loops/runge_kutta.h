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
  auto k1 = fn(X);
  auto k2 = fn(X + half_dt * k1);
  auto k3 = fn(X + half_dt * k2);
  auto k4 = fn(X + dt * k3);
  return X + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_RUNGE_KUTTA_H_
