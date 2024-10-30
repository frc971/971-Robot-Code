#ifndef FRC971_CONTROL_LOOPS_DLQR_H_
#define FRC971_CONTROL_LOOPS_DLQR_H_

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>
#include <tl/expected.hpp>

#include "frc971/control_loops/dare.h"

namespace frc971::controls {

template <typename Scalar, int num_states, int num_inputs>
int Controllability(const ::Eigen::Matrix<Scalar, num_states, num_states> &A,
                    const ::Eigen::Matrix<Scalar, num_states, num_inputs> &B) {
  Eigen::Matrix<Scalar, num_states, num_states * num_inputs> controllability;
  controllability.block(0, 0, num_states, num_inputs) = B;

  for (size_t i = 1; i < num_states; i++) {
    controllability.block(0, i * num_inputs, num_states, num_inputs) =
        A *
        controllability.block(0, (i - 1) * num_inputs, num_states, num_inputs);
  }

  return Eigen::FullPivLU<
             Eigen::Matrix<Scalar, num_states, num_states * num_inputs>>(
             controllability)
      .rank();
}

/**
 * Computes the optimal discrete LQR controller gain K for the provided system
 * and costs.
 *
 * @param A The system matrix.
 * @param B The input matrix.
 * @param Q The state cost matrix.
 * @param R The input cost matrix.
 * @param check_preconditions Whether to check preconditions (30% less time if
 *   user is sure preconditions are already met).
 * @return The controller gain K on success, or DAREError on failure.
 */
template <int num_states, int num_inputs>
tl::expected<Eigen::Matrix<double, num_inputs, num_states>, DAREError> dlqr(
    const Eigen::Matrix<double, num_states, num_states> &A,
    const Eigen::Matrix<double, num_states, num_inputs> &B,
    const Eigen::Matrix<double, num_states, num_states> &Q,
    const Eigen::Matrix<double, num_inputs, num_inputs> &R,
    bool check_preconditions = true) {
  if (auto X = dare<num_states, num_inputs>(A, B, Q, R, check_preconditions)) {
    // K = (BᵀSB + R)⁻¹BᵀSA
    return (B.transpose() * X.value() * B + R)
        .llt()
        .solve(B.transpose() * X.value() * A);
  } else {
    return tl::unexpected{X.error()};
  }
}

}  // namespace frc971::controls

#endif  // FRC971_CONTROL_LOOPS_DLQR_H_
