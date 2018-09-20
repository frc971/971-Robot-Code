#ifndef FRC971_CONTROL_LOOPS_DLQR_H_
#define FRC971_CONTROL_LOOPS_DLQR_H_

#include <Eigen/Dense>


namespace frc971 {
namespace controls {

extern "C" {
// Forward declaration for slicot fortran library.
void sb02od_(char *DICO, char *JOBB, char *FACT, char *UPLO, char *JOBL,
            char *SORT, int *N, int *M, int *P, double *A, int *LDA, double *B,
            int *LDB, double *Q, int *LDQ, double *R, int *LDR, double *L,
            int *LDL, double *RCOND, double *X, int *LDX, double *ALFAR,
            double *ALFAI, double *BETA, double *S, int *LDS, double *T,
            int *LDT, double *U, int *LDU, double *TOL, int *IWORK,
            double *DWORK, int *LDWORK, int *BWORK, int *INFO);
}

template <int kN, int kM>
void dlqr(::Eigen::Matrix<double, kN, kN> A, ::Eigen::Matrix<double, kN, kM> B,
          ::Eigen::Matrix<double, kN, kN> Q, ::Eigen::Matrix<double, kM, kM> R,
          ::Eigen::Matrix<double, kM, kN> *K,
          ::Eigen::Matrix<double, kN, kN> *S) {
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

  int N = 4;
  int M = 2;
  // Not needed since FACT = N
  int P = 0;

  int LDL = 1;

  double RCOND = 0.0;
  ::Eigen::Matrix<double, kN, kN> X = ::Eigen::Matrix<double, kN, kN>::Zero();

  double ALFAR[kN * 2];
  double ALFAI[kN * 2];
  double BETA[kN * 2];
  memset(ALFAR, 0, kN * 2);
  memset(ALFAI, 0, kN * 2);
  memset(BETA, 0, kN * 2);

  int LDS = 2 * kN + kM;
  ::Eigen::Matrix<double, 2 * kN + kM, 2 *kN + kM> S_schur =
      ::Eigen::Matrix<double, 2 * kN + kM, 2 * kN + kM>::Zero();

  ::Eigen::Matrix<double, 2 * kN + kM, 2 *kN> T =
      ::Eigen::Matrix<double, 2 * kN + kM, 2 * kN>::Zero();
  int LDT = 2 * kN + kM;

  ::Eigen::Matrix<double, 2 * kN, 2 *kN> U =
      ::Eigen::Matrix<double, 2 * kN, 2 * kN>::Zero();
  int LDU = 2 * kN;

  double TOL = 0.0;

  int IWORK[2 * kN > kM ? 2 * kN : kM];
  memset(IWORK, 0, 2 * kN > kM ? 2 * kN : kM);

  int LDWORK = 16 * kN + 3 * kM + 16;

  double DWORK[LDWORK];
  memset(DWORK, 0, LDWORK);

  int INFO = 0;

  int BWORK[2 * kN];
  memset(BWORK, 0, 2 * kN);

  // TODO(austin): I can't tell if anything here is transposed...
  sb02od_(&DICO, &JOBB, &FACT, &UPLO, &JOBL, &SORT, &N, &M, &P, A.data(),
          &N, B.data(), &N, Q.data(), &N, R.data(), &M, nullptr, &LDL,
          &RCOND, X.data(), &N, ALFAR, ALFAI, BETA, S_schur.data(), &LDS,
          T.data(), &LDT, U.data(), &LDU, &TOL, IWORK, DWORK, &LDWORK, BWORK,
          &INFO);
  //::std::cout << "slycot result: " << INFO << ::std::endl;

  *K = (R + B.transpose() * X * B).inverse() * B.transpose() * X * A;
  if (S != nullptr) {
    *S = X;
  }
}

}  // namespace controls
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DLQR_H_
