#ifndef FRC971_CONTROL_LOOPS_C2D_H_
#define FRC971_CONTROL_LOOPS_C2D_H_

#include <chrono>

#include <Eigen/Dense>

namespace frc971 {
namespace controls {

template <typename Scalar, int num_states, int num_inputs>
void C2D(const ::Eigen::Matrix<Scalar, num_states, num_states> &A_continuous,
         const ::Eigen::Matrix<Scalar, num_states, num_inputs> &B_continuous,
         ::std::chrono::nanoseconds dt,
         ::Eigen::Matrix<Scalar, num_states, num_states> *A,
         ::Eigen::Matrix<Scalar, num_states, num_inputs> *B) {
  // Trick from
  // https://en.wikipedia.org/wiki/Discretization#Discretization_of_linear_state_space_models
  // to solve for A and B more efficiently.
  Eigen::Matrix<Scalar, num_states + num_inputs, num_states + num_inputs>
      M_state_continuous;
  M_state_continuous.setZero();
  M_state_continuous.template block<num_states, num_states>(0, 0) =
      A_continuous *
      ::std::chrono::duration_cast<::std::chrono::duration<Scalar>>(dt).count();
  M_state_continuous.template block<num_states, num_inputs>(0, num_states) =
      B_continuous *
      ::std::chrono::duration_cast<::std::chrono::duration<Scalar>>(dt).count();

  Eigen::Matrix<Scalar, num_states + num_inputs, num_states + num_inputs>
      M_state = M_state_continuous.exp();
  *A = M_state.template block<num_states, num_states>(0, 0);
  *B = M_state.template block<num_states, num_inputs>(0, num_states);
}

}  // namespace controls
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_C2D_H_
