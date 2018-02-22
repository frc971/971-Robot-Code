#ifndef FRC971_CONTROL_LOOPS_JACOBIAN_H_
#define FRC971_CONTROL_LOOPS_JACOBIAN_H_

#include <Eigen/Dense>

namespace frc971 {
namespace control_loops {

template <int num_states, int num_inputs, typename F>
::Eigen::Matrix<double, num_states, num_inputs> NumericalJacobian(
    const F &fn, ::Eigen::Matrix<double, num_inputs, 1> input) {
  constexpr double kEpsilon = 1e-5;
  ::Eigen::Matrix<double, num_states, num_inputs> result =
      ::Eigen::Matrix<double, num_states, num_inputs>::Zero();

  // It's more expensive, but +- epsilon will be more accurate
  for (int i = 0; i < num_inputs; ++i) {
    ::Eigen::Matrix<double, num_inputs, 1> dX_plus = input;
    dX_plus(i, 0) += kEpsilon;
    ::Eigen::Matrix<double, num_inputs, 1> dX_minus = input;
    dX_minus(i, 0) -= kEpsilon;
    result.col(i) = (fn(dX_plus) - fn(dX_minus)) / (kEpsilon * 2.0);
  }
  return result;
}

// Implements a numerical jacobian with respect to X for f(X, U, ...).
template <int num_states, int num_u, typename F, typename... Args>
::Eigen::Matrix<double, num_states, num_states> NumericalJacobianX(
    const F &fn, ::Eigen::Matrix<double, num_states, 1> X,
    ::Eigen::Matrix<double, num_u, 1> U, Args &&... args) {
  return NumericalJacobian<num_states, num_states>(
      [&](::Eigen::Matrix<double, num_states, 1> X) {
        return fn(X, U, args...);
      },
      X);
}

// Implements a numerical jacobian with respect to U for f(X, U, ...).
template <int num_states, int num_u, typename F, typename... Args>
::Eigen::Matrix<double, num_states, num_u> NumericalJacobianU(
    const F &fn, ::Eigen::Matrix<double, num_states, 1> X,
    ::Eigen::Matrix<double, num_u, 1> U, Args &&... args) {
  return NumericalJacobian<num_states, num_u>(
      [&](::Eigen::Matrix<double, num_u, 1> U) { return fn(X, U, args...); },
      U);
}

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_JACOBIAN_H_
