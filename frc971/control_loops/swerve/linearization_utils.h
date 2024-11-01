#ifndef FRC971_CONTROL_LOOPS_SWERVE_LINEARIZATION_UTILS_H_
#define FRC971_CONTROL_LOOPS_SWERVE_LINEARIZATION_UTILS_H_

#include "frc971/control_loops/jacobian.h"

namespace frc971::control_loops::swerve {
template <typename Scalar, int kNumStates, int kNumInputs>
struct DynamicsInterface {
  typedef Eigen::Matrix<Scalar, kNumStates, 1> State;
  typedef Eigen::Matrix<Scalar, kNumInputs, 1> Input;
  typedef Eigen::Matrix<Scalar, kNumStates, kNumStates> StateSquare;
  typedef Eigen::Matrix<Scalar, kNumStates, kNumInputs> BMatrix;
  // Represents the linearized dynamics of the system xdot = A * x + B * u
  struct LinearDynamics {
    StateSquare A;
    BMatrix B;
  };
  virtual ~DynamicsInterface() {}
  // To be overridden by the implementation; returns the derivative of the state
  // at the given state with the provided control input.
  virtual State operator()(const State &X, const Input &U) const = 0;

  virtual LinearDynamics LinearizeDynamics(const State &X,
                                           const Input &U) const {
    return {.A = NumericalJacobianX(*this, X, U),
            .B = NumericalJacobianU(*this, X, U)};
  }
};
}  // namespace frc971::control_loops::swerve
#endif  // FRC971_CONTROL_LOOPS_SWERVE_LINEARIZATION_UTILS_H_
