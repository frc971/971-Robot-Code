#ifndef FRC971_CONTROL_LOOPS_DARE_H_
#define FRC971_CONTROL_LOOPS_DARE_H_

#include "Eigen/Cholesky"
#include "Eigen/Core"
#include "Eigen/Eigenvalues"
#include "Eigen/LU"
#include "Eigen/QR"
#include "absl/log/log.h"
#include "tl/expected.hpp"

namespace frc971::controls {

/**
 * Returns true if (A, B) is a stabilizable pair.
 *
 * (A, B) is stabilizable if and only if the uncontrollable eigenvalues of A, if
 * any, have norm less than one, where an eigenvalue is uncontrollable if
 * rank([λI - A, B]) < n where n is the number of states (see
 * https://en.m.wikipedia.org/wiki/Hautus_lemma). The norm check is the discrete
 * translation of the continous time check where the real part >= 0 in the
 * wikipedia page. This is in essence whether the matrix itself is stable.
 *
 * @tparam num_states Number of states.
 * @tparam num_inputs Number of inputs.
 * @param A System matrix.
 * @param B Input matrix.
 */
template <typename Scalar, int num_states, int num_inputs>
bool IsStabilizable(const Eigen::Matrix<Scalar, num_states, num_states> &A,
                    const Eigen::Matrix<Scalar, num_states, num_inputs> &B) {
  Eigen::EigenSolver<Eigen::Matrix<Scalar, num_states, num_states>> es{A,
                                                                       false};

  for (int i = 0; i < A.rows(); ++i) {
    if (std::norm(es.eigenvalues()[i]) < 1) {
      continue;
    }

    Eigen::Matrix<std::complex<Scalar>, num_states, num_states + num_inputs> E;
    E << es.eigenvalues()[i] * Eigen::Matrix<std::complex<Scalar>, num_states,
                                             num_states>::Identity() -
             A,
        B;

    Eigen::ColPivHouseholderQR<Eigen::Matrix<std::complex<Scalar>, num_states,
                                             num_states + num_inputs>>
        qr{E};
    if (qr.rank() < num_states) {
      return false;
    }
  }
  return true;
}

/**
 * Returns true if (A, C) is a detectable pair.
 *
 * (A, C) is detectable if and only if the unobservable eigenvalues of A, if
 * any, have absolute values less than one, where an eigenvalue is unobservable
 * if rank([λI - A; C]) < n where n is the number of states. The system (C, A)
 * is detectable if (A', C') is stabilizable.
 *
 * @tparam num_states Number of states.
 * @tparam num_outputs Number of outputs.
 * @param A System matrix.
 * @param C Output matrix.
 */
template <typename Scalar, int num_states, int num_outputs>
bool IsDetectable(const Eigen::Matrix<Scalar, num_states, num_states> &A,
                  const Eigen::Matrix<Scalar, num_outputs, num_states> &C) {
  return IsStabilizable<Scalar, num_states, num_outputs>(A.transpose(),
                                                         C.transpose());
}

/**
 * Returns true if A, a square matrix, is symmetric
 *
 * A is symmetric if the itself minus the tranpose is equal to 0.
 *
 * @tparam num_states Number of columns or rows.
 * @param A matrix to be tested.
 */
template <typename Scalar, int num_states>
bool IsSymmetric(const Eigen::Matrix<Scalar, num_states, num_states> &A) {
  return (A - A.transpose()).norm() < 1e-10;
}

/**
 * Returns true if R, a square matrix, is positive definite
 *
 * If the Cholesky Decomposition is successful then it is positive definite.
 *
 * @tparam num_states Number of columns or rows.
 * @param R the matrix to check.
 */
template <typename Scalar, int num_states>
bool IsPositiveDefinite(
    const Eigen::Matrix<Scalar, num_states, num_states> &R) {
  auto R_llt = R.llt();
  return R_llt.info() == Eigen::Success;
}

/**
 * Returns true if Q, a square matrix, is positive semi-definite
 *
 * Checks using the Sylvester Criterion (see
 * https://en.wikipedia.org/wiki/Sylvester's_criterion). It's positive
 * semi-definite if all the principle minors are nonnegative.
 *
 * @tparam num_states Number of columns or rows.
 * @param Q the matrix to check.
 */
template <typename Scalar, int num_states>
bool IsPositiveSemiDefinite(
    const Eigen::Matrix<Scalar, num_states, num_states> &Q) {
  auto Q_ldlt = Q.ldlt();
  return Q_ldlt.info() == Eigen::Success &&
         (Q_ldlt.vectorD().array() >= 0.0).all();
}

/**
 * Errors the DARE solver can encounter.
 */
enum class DareError {
  /// Q was not symmetric.
  QNotSymmetric,
  /// Q was not positive semidefinite.
  QNotPositiveSemidefinite,
  /// R was not symmetric.
  RNotSymmetric,
  /// R was not positive definite.
  RNotPositiveDefinite,
  /// (A, B) pair was not stabilizable.
  ABNotStabilizable,
  /// (A, C) pair where Q = CᵀC was not detectable.
  ACNotDetectable,
};

/**
 * Allows DareError to be used with LOG(INFO) and absl
 */
template <typename Sink>
void AbslStringify(Sink &sink, DareError e) {
  switch (e) {
    case DareError::QNotSymmetric:
      sink.Append("Q was not symmetric.");
      break;
    case DareError::QNotPositiveSemidefinite:
      sink.Append("Q was not positive semidefinite.");
      break;
    case DareError::RNotSymmetric:
      sink.Append("R was not symmetric.");
      break;
    case DareError::RNotPositiveDefinite:
      sink.Append("R was not positive definite.");
      break;
    case DareError::ABNotStabilizable:
      sink.Append("(A, B) pair was not stabilizable.");
      break;
    case DareError::ACNotDetectable:
      sink.Append("(A, C) pair where Q = CᵀC was not detectable.");
      break;
  }
}

/**
 * Computes the unique stabilizing solution X to the discrete-time algebraic
 * Riccati equation:
 *
 *   AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0
 *
 * @tparam num_states Number of states.
 * @tparam num_inputs Number of inputs.
 * @param A The system matrix.
 * @param B The input matrix.
 * @param Q The state cost matrix.
 * @param R The input cost matrix.
 * @param check_preconditions Whether to check preconditions (30% less time if
 *   user is sure preconditions are already met).
 * @param max_iters The max number of iterations for the DARE solver. A very
 * nearly unstabilizable solution at machine precision would take aroun 50-60
 * iterations. This was found setting the d variable in the python test to
 * machine precision.
 * @return Solution to the DARE on success, or DareError on failure.
 */
template <typename Scalar, int num_states, int num_inputs>
tl::expected<Eigen::Matrix<Scalar, num_states, num_states>, DareError> dare(
    const Eigen::Matrix<Scalar, num_states, num_states> &A,
    const Eigen::Matrix<Scalar, num_states, num_inputs> &B,
    const Eigen::Matrix<Scalar, num_states, num_states> &Q,
    const Eigen::Matrix<Scalar, num_inputs, num_inputs> &R,
    bool check_preconditions = true, int max_iters = 50) {
  using StateMatrix = Eigen::Matrix<Scalar, num_states, num_states>;

  if (check_preconditions) {
    // Require R be symmetric
    if (!IsSymmetric<Scalar, num_inputs>(R)) {
      return tl::unexpected{DareError::RNotSymmetric};
    }

    // Require R be positive definite
    if (!IsPositiveDefinite<Scalar, num_inputs>(R)) {
      return tl::unexpected{DareError::RNotPositiveDefinite};
    }

    // Require Q be symmetric
    if (!IsSymmetric<Scalar, num_states>(Q)) {
      return tl::unexpected{DareError::QNotSymmetric};
    }

    // Require Q be positive semidefinite
    //
    // If Q is a symmetric matrix with a decomposition LDLᵀ, the number of
    // positive, negative, and zero diagonal entries in D equals the number of
    // positive, negative, and zero eigenvalues respectively in Q (see
    // https://en.wikipedia.org/wiki/Sylvester's_law_of_inertia).
    //
    // Therefore, D having no negative diagonal entries is sufficient to prove Q
    // is positive semidefinite.
    if (!IsPositiveSemiDefinite<Scalar, num_states>(Q)) {
      return tl::unexpected{DareError::QNotPositiveSemidefinite};
    }

    // Require (A, B) pair be stabilizable
    if (!IsStabilizable<Scalar, num_states, num_inputs>(A, B)) {
      return tl::unexpected{DareError::ABNotStabilizable};
    }

    // Require (A, C) pair be detectable where Q = CᵀC
    //
    // Q = CᵀC = PᵀLDLᵀP
    // C = √(D)LᵀP
    auto Q_ldlt = Q.ldlt();
    StateMatrix C = Q_ldlt.vectorD().cwiseSqrt().asDiagonal() *
                    StateMatrix{Q_ldlt.matrixL().transpose()} *
                    Q_ldlt.transpositionsP();

    if (!IsDetectable<Scalar, num_states, num_states>(A, C)) {
      return tl::unexpected{DareError::ACNotDetectable};
    }
  }

  // Implements SDA algorithm on p. 5 of [1] (initial A, G, H are from (4)).
  //
  // The paper (and we) make the following assumptions
  // 1. S = 0.
  // 2. All matrices are time-invariant.
  // 3. Q (in paper terminology) is the identity matrix.
  // 4. Q (in our terminology) is C^T C in the paper's terminology, hence why we
  // must calculate C.
  //
  // [1] E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang "Structure-Preserving
  //     Algorithms for Periodic Discrete-Time Algebraic Riccati Equations",
  //     International Journal of Control, 77:8, 767-788, 2004.
  //     DOI: 10.1080/00207170410001714988

  // A₀ = A
  // G₀ = BR⁻¹Bᵀ
  // H₀ = Q
  StateMatrix A_k = A;
  StateMatrix G_k = B * R.llt().solve(B.transpose());
  StateMatrix H_k;
  StateMatrix H_k1 = Q;

  int loop_count = 0;
  do {
    H_k = H_k1;

    // W = I + GₖHₖ
    StateMatrix W = StateMatrix::Identity() + G_k * H_k;

    auto W_solver = W.lu();

    // Solve WV₁ = Aₖ for V₁
    // Solve V₂Wᵀ = Gₖ for V₂
    StateMatrix V_1 = W_solver.solve(A_k);
    StateMatrix V_2 = W_solver.solve(G_k.transpose()).transpose();

    // Gₖ₊₁ = Gₖ + AₖV₂Aₖᵀ
    // Hₖ₊₁ = Hₖ + V₁ᵀHₖAₖ
    // Aₖ₊₁ = AₖV₁
    G_k += A_k * V_2 * A_k.transpose();
    H_k1 = H_k + V_1.transpose() * H_k * A_k;
    A_k = A_k * V_1;

    // while |Hₖ₊₁ − Hₖ| > ε |Hₖ₊₁|
  } while ((H_k1 - H_k).norm() > 1e-10 * H_k1.norm() &&
           ++loop_count < max_iters);
  VLOG(3) << "loop_count: " << loop_count;
  return H_k1;
}

}  // namespace frc971::controls

#endif  // FRC971_CONTROL_LOOPS_DARE_H_
