#ifndef FRC971_CONTROL_LOOPS_DARE_H_
#define FRC971_CONTROL_LOOPS_DARE_H_

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/QR>
#include <tl/expected.hpp>

namespace frc971::controls {

/**
 * Returns true if (A, B) is a stabilizable pair.
 *
 * (A, B) is stabilizable if and only if the uncontrollable eigenvalues of A, if
 * any, have absolute values less than one, where an eigenvalue is
 * uncontrollable if rank([λI - A, B]) < n where n is the number of states.
 *
 * @tparam num_states Number of states.
 * @tparam num_inputs Number of inputs.
 * @param A System matrix.
 * @param B Input matrix.
 */
template <int num_states, int num_inputs>
bool is_stabilizable(const Eigen::Matrix<double, num_states, num_states> &A,
                     const Eigen::Matrix<double, num_states, num_inputs> &B) {
  Eigen::EigenSolver<Eigen::Matrix<double, num_states, num_states>> es{A,
                                                                       false};

  for (int i = 0; i < A.rows(); ++i) {
    if (std::norm(es.eigenvalues()[i]) < 1) {
      continue;
    }

    Eigen::Matrix<std::complex<double>, num_states, num_states + num_inputs> E;
    E << es.eigenvalues()[i] * Eigen::Matrix<std::complex<double>, num_states,
                                             num_states>::Identity() -
             A,
        B;

    Eigen::ColPivHouseholderQR<Eigen::Matrix<std::complex<double>, num_states,
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
 * if rank([λI - A; C]) < n where n is the number of states.
 *
 * @tparam num_states Number of states.
 * @tparam num_outputs Number of outputs.
 * @param A System matrix.
 * @param C Output matrix.
 */
template <int num_states, int num_outputs>
bool is_detectable(const Eigen::Matrix<double, num_states, num_states> &A,
                   const Eigen::Matrix<double, num_outputs, num_states> &C) {
  return is_stabilizable<num_states, num_outputs>(A.transpose(), C.transpose());
}

/**
 * Errors the DARE solver can encounter.
 */
enum class DAREError {
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
 * Converts the given DAREError enum to a string.
 */
constexpr const char *to_string(const DAREError &error) {
  switch (error) {
    case DAREError::QNotSymmetric:
      return "Q was not symmetric.";
    case DAREError::QNotPositiveSemidefinite:
      return "Q was not positive semidefinite.";
    case DAREError::RNotSymmetric:
      return "R was not symmetric.";
    case DAREError::RNotPositiveDefinite:
      return "R was not positive definite.";
    case DAREError::ABNotStabilizable:
      return "(A, B) pair was not stabilizable.";
    case DAREError::ACNotDetectable:
      return "(A, C) pair where Q = CᵀC was not detectable.";
  }

  return "";
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
 * @return Solution to the DARE on success, or DAREError on failure.
 */
template <int num_states, int num_inputs>
tl::expected<Eigen::Matrix<double, num_states, num_states>, DAREError> dare(
    const Eigen::Matrix<double, num_states, num_states> &A,
    const Eigen::Matrix<double, num_states, num_inputs> &B,
    const Eigen::Matrix<double, num_states, num_states> &Q,
    const Eigen::Matrix<double, num_inputs, num_inputs> &R,
    bool check_preconditions = true) {
  using StateMatrix = Eigen::Matrix<double, num_states, num_states>;

  if (check_preconditions) {
    // Require R be symmetric
    if ((R - R.transpose()).norm() > 1e-10) {
      return tl::unexpected{DAREError::RNotSymmetric};
    }
  }

  // Require R be positive definite
  auto R_llt = R.llt();
  if (R_llt.info() != Eigen::Success) {
    return tl::unexpected{DAREError::RNotPositiveDefinite};
  }

  if (check_preconditions) {
    // Require Q be symmetric
    if ((Q - Q.transpose()).norm() > 1e-10) {
      return tl::unexpected{DAREError::QNotSymmetric};
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
    auto Q_ldlt = Q.ldlt();
    if (Q_ldlt.info() != Eigen::Success ||
        (Q_ldlt.vectorD().array() < 0.0).any()) {
      return tl::unexpected{DAREError::QNotPositiveSemidefinite};
    }

    // Require (A, B) pair be stabilizable
    if (!is_stabilizable<num_states, num_inputs>(A, B)) {
      return tl::unexpected{DAREError::ABNotStabilizable};
    }

    // Require (A, C) pair be detectable where Q = CᵀC
    //
    // Q = CᵀC = PᵀLDLᵀP
    // C = √(D)LᵀP
    StateMatrix C = Q_ldlt.vectorD().cwiseSqrt().asDiagonal() *
                    StateMatrix{Q_ldlt.matrixL().transpose()} *
                    Q_ldlt.transpositionsP();

    if (!is_detectable<num_states, num_states>(A, C)) {
      return tl::unexpected{DAREError::ACNotDetectable};
    }
  }

  // Implements SDA algorithm on p. 5 of [1] (initial A, G, H are from (4)).
  //
  // [1] E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang "Structure-Preserving
  //     Algorithms for Periodic Discrete-Time Algebraic Riccati Equations",
  //     International Journal of Control, 77:8, 767-788, 2004.
  //     DOI: 10.1080/00207170410001714988

  // A₀ = A
  // G₀ = BR⁻¹Bᵀ
  // H₀ = Q
  StateMatrix A_k = A;
  StateMatrix G_k = B * R_llt.solve(B.transpose());
  StateMatrix H_k;
  StateMatrix H_k1 = Q;

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
    A_k *= V_1;

    // while |Hₖ₊₁ − Hₖ| > ε |Hₖ₊₁|
  } while ((H_k1 - H_k).norm() > 1e-10 * H_k1.norm());

  return H_k1;
}

}  // namespace frc971::controls

#endif  // FRC971_CONTROL_LOOPS_DARE_H_
