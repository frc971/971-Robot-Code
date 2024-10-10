#ifndef FRC971_CONTROL_LOOPS_SWERVE_LINEARIZATION_UTILS_H_
#define FRC971_CONTROL_LOOPS_SWERVE_LINEARIZATION_UTILS_H_

namespace frc971::control_loops::swerve {
template <typename State, typename Input>
struct DynamicsInterface {
  virtual ~DynamicsInterface() {}
  // To be overridden by the implementation; returns the derivative of the state
  // at the given state with the provided control input.
  virtual State operator()(const State &X, const Input &U) const = 0;
};
}  // namespace frc971::control_loops::swerve
#endif  // FRC971_CONTROL_LOOPS_SWERVE_LINEARIZATION_UTILS_H_
