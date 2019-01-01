#ifndef FRC971_CONTROL_LOOPS_DLQR_H_
#define FRC971_CONTROL_LOOPS_DLQR_H_

#include <Eigen/Dense>

namespace frc971 {
namespace controls {

template <int num_states, int num_inputs>
int Controllability(const ::Eigen::Matrix<double, num_states, num_states> &A,
                    const ::Eigen::Matrix<double, num_states, num_inputs> &B) {
  Eigen::Matrix<double, num_states, num_states * num_inputs> controllability;
  controllability.block(0, 0, num_states, num_inputs) = B;

  for (size_t i = 1; i < num_states; i++) {
    controllability.block(0, i * num_inputs, num_states, num_inputs) =
        A *
        controllability.block(0, (i - 1) * num_inputs, num_states, num_inputs);
  }

  return Eigen::FullPivLU<
             Eigen::Matrix<double, num_states, num_states * num_inputs>>(
             controllability)
      .rank();
}

extern "C" {
// Forward declaration for slicot fortran library.
void sb02od_(char *DICO, char *JOBB, char *FACT, char *UPLO, char *JOBL,
             char *SORT, long *N, long *M, long *P, double *A, long *LDA,
             double *B, long *LDB, double *Q, long *LDQ, double *R, long *LDR,
             double *L, long *LDL, double *RCOND, double *X, long *LDX,
             double *ALFAR, double *ALFAI, double *BETA, double *S, long *LDS,
             double *T, long *LDT, double *U, long *LDU, double *TOL,
             long *IWORK, double *DWORK, long *LDWORK, long *BWORK, long *INFO);
}

// Computes the optimal LQR controller K for the provided system and costs.
template <int kN, int kM>
int dlqr(::Eigen::Matrix<double, kN, kN> A, ::Eigen::Matrix<double, kN, kM> B,
          ::Eigen::Matrix<double, kN, kN> Q, ::Eigen::Matrix<double, kM, kM> R,
          ::Eigen::Matrix<double, kM, kN> *K,
          ::Eigen::Matrix<double, kN, kN> *S) {
  *K = ::Eigen::Matrix<double, kM, kN>::Zero();
  *S = ::Eigen::Matrix<double, kN, kN>::Zero();
  // Discrete (not continuous)
  char DICO = 'D';
  // B and R are provided instead of G.
  char JOBB = 'B';
  // Not factored, Q and R are provide.
  char FACT = 'N';
  // Store the upper triangle of Q and R.
  char UPLO = 'U';
  // L is zero.
  char JOBL = 'Z';
  // Stable eigenvalues first in the sort order
  char SORT = 'S';

  long N = kN;
  long M = kM;
  // Not needed since FACT = N
  long P = 0;

  long LDL = 1;

  double RCOND = 0.0;
  ::Eigen::Matrix<double, kN, kN> X = ::Eigen::Matrix<double, kN, kN>::Zero();

  double ALFAR[kN * 2];
  double ALFAI[kN * 2];
  double BETA[kN * 2];
  memset(ALFAR, 0, kN * 2);
  memset(ALFAI, 0, kN * 2);
  memset(BETA, 0, kN * 2);

  long LDS = 2 * kN + kM;
  ::Eigen::Matrix<double, 2 * kN + kM, 2 *kN + kM> S_schur =
      ::Eigen::Matrix<double, 2 * kN + kM, 2 * kN + kM>::Zero();

  ::Eigen::Matrix<double, 2 * kN + kM, 2 *kN> T =
      ::Eigen::Matrix<double, 2 * kN + kM, 2 * kN>::Zero();
  long LDT = 2 * kN + kM;

  ::Eigen::Matrix<double, 2 * kN, 2 *kN> U =
      ::Eigen::Matrix<double, 2 * kN, 2 * kN>::Zero();
  long LDU = 2 * kN;

  double TOL = 0.0;

  long IWORK[2 * kN > kM ? 2 * kN : kM];
  memset(IWORK, 0, 2 * kN > kM ? 2 * kN : kM);

  long LDWORK = 16 * kN + 3 * kM + 16;

  double DWORK[LDWORK];
  memset(DWORK, 0, LDWORK);

  long INFO = 0;

  long BWORK[2 * kN];
  memset(BWORK, 0, 2 * kN);

  // TODO(austin): I can't tell if anything here is transposed...
  sb02od_(&DICO, &JOBB, &FACT, &UPLO, &JOBL, &SORT, &N, &M, &P, A.data(), &N,
          B.data(), &N, Q.data(), &N, R.data(), &M, nullptr, &LDL, &RCOND,
          X.data(), &N, ALFAR, ALFAI, BETA, S_schur.data(), &LDS, T.data(),
          &LDT, U.data(), &LDU, &TOL, IWORK, DWORK, &LDWORK, BWORK, &INFO);

  *K = (R + B.transpose() * X * B).inverse() * B.transpose() * X * A;
  if (S != nullptr) {
    *S = X;
  }

  return INFO;
}

}  // namespace controls
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DLQR_H_
