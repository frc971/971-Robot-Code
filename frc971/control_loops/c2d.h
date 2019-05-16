#ifndef FRC971_CONTROL_LOOPS_C2D_H_
#define FRC971_CONTROL_LOOPS_C2D_H_

#include <chrono>

#include <Eigen/Dense>
#include "aos/time/time.h"
// We need to include MatrixFunctions for the matrix exponential.
#include "unsupported/Eigen/MatrixFunctions"

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
      A_continuous * ::aos::time::TypedDurationInSeconds<Scalar>(dt);
  M_state_continuous.template block<num_states, num_inputs>(0, num_states) =
      B_continuous * ::aos::time::TypedDurationInSeconds<Scalar>(dt);

  Eigen::Matrix<Scalar, num_states + num_inputs, num_states + num_inputs>
      M_state = M_state_continuous.exp();
  *A = M_state.template block<num_states, num_states>(0, 0);
  *B = M_state.template block<num_states, num_inputs>(0, num_states);
}

template <typename Scalar, int num_states>
void DiscretizeQ(
    const Eigen::Matrix<Scalar, num_states, num_states> &Q_continuous,
    const Eigen::Matrix<Scalar, num_states, num_states> &A_continuous,
    ::std::chrono::nanoseconds dt,
    Eigen::Matrix<Scalar, num_states, num_states> *Q_d) {
  Eigen::Matrix<Scalar, num_states, num_states> Qtemp =
      (Q_continuous + Q_continuous.transpose()) / static_cast<Scalar>(2.0);
  Eigen::Matrix<Scalar, 2 * num_states, 2 * num_states> M_gain;
  M_gain.setZero();
  // Set up the matrix M = [[-A, Q], [0, A.T]]
  M_gain.template block<num_states, num_states>(0, 0) = -A_continuous;
  M_gain.template block<num_states, num_states>(0, num_states) = Qtemp;
  M_gain.template block<num_states, num_states>(num_states, num_states) =
      A_continuous.transpose();

  Eigen::Matrix<Scalar, 2 * num_states, 2 *num_states> phi =
      (M_gain * ::aos::time::TypedDurationInSeconds<Scalar>(dt)).exp();

  // Phi12 = phi[0:num_states, num_states:2*num_states]
  // Phi22 = phi[num_states:2*num_states,
  // num_states:2*num_states]
  Eigen::Matrix<Scalar, num_states, num_states> phi12 =
      phi.block(0, num_states, num_states, num_states);
  Eigen::Matrix<Scalar, num_states, num_states> phi22 =
      phi.block(num_states, num_states, num_states, num_states);

  Qtemp = phi22.transpose() * phi12;
  *Q_d = (Qtemp + Qtemp.transpose()) / static_cast<Scalar>(2.0);
}

// Does a faster approximation for the discretizing A/Q, for if solving a 2Nx2N
// matrix exponential is too expensive.
// Basic reasoning/method:
// The original algorithm does a matrix exponential on a 2N x 2N matrix (the
// block matrix made of of A and Q). This is extremely expensive for larg-ish
// matrices. This function takes advantage of the structure of the matrix
// we are taking the exponential and notes that we care about two things:
// 1) The exponential of A*t, which is only NxN and so is relatively cheap.
// 2) The upper-right quarter of the 2Nx2N matrix, which we can approximate
//    using a taylor series to several terms and still be substantially cheaper
//    than taking the big exponential.
template <typename Scalar, int num_states>
void DiscretizeQAFast(
    const Eigen::Matrix<Scalar, num_states, num_states> &Q_continuous,
    const Eigen::Matrix<Scalar, num_states, num_states> &A_continuous,
    ::std::chrono::nanoseconds dt,
    Eigen::Matrix<Scalar, num_states, num_states> *Q_d,
    Eigen::Matrix<Scalar, num_states, num_states> *A_d) {
  Eigen::Matrix<Scalar, num_states, num_states> Qtemp =
      (Q_continuous + Q_continuous.transpose()) / static_cast<Scalar>(2.0);
  Scalar dt_d = ::aos::time::TypedDurationInSeconds<Scalar>(dt);
  Eigen::Matrix<Scalar, num_states, num_states> last_term = Qtemp;
  double last_coeff = dt_d;
  const Eigen::Matrix<Scalar, num_states, num_states> At =
      A_continuous.transpose();
  Eigen::Matrix<Scalar, num_states, num_states> Atn = At;
  Eigen::Matrix<Scalar, num_states, num_states> phi12 = last_term * last_coeff;
  Eigen::Matrix<Scalar, num_states, num_states> phi22 =
      At.Identity() + Atn * last_coeff;
  // TODO(james): Tune this once we have the robot up; ii < 6 is enough to get
  // beyond any real numerical issues. 5 should be fine, but just happened to
  // kick a test over to failing. 4 would probably work.
  for (int ii = 2; ii < 6; ++ii) {
    Eigen::Matrix<Scalar, num_states, num_states> next_term =
        -A_continuous * last_term + Qtemp * Atn;
    last_coeff *= dt_d / static_cast<Scalar>(ii);
    phi12 += next_term * last_coeff;

    last_term = next_term;

    Atn *= At;
    phi22 += last_coeff * Atn;
  }
  Eigen::Matrix<Scalar, num_states, num_states> phi22t = phi22.transpose();

  Qtemp = phi22t * phi12;
  *Q_d = (Qtemp + Qtemp.transpose()) / static_cast<Scalar>(2.0);
  *A_d = phi22t;
}

}  // namespace controls
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_C2D_H_
