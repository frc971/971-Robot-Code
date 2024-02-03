#include "frc971/control_loops/catapult/mpc_problem.h"
#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {
namespace catapult {

// Decently efficient problem generator for multiple horizons given a max
// horizon to solve for.
//
// The math is documented in mpc.tex
class CatapultProblemGenerator {
 public:
  // Builds a problem generator for the specified max horizon and caches a lot
  // of the state.
  CatapultProblemGenerator(StateFeedbackPlant<2, 1, 1> plant, size_t horizon);

  // Returns the maximum horizon.
  size_t horizon() const { return horizon_; }

  // Makes a problem for the specificed horizon.
  std::unique_ptr<MPCProblem> MakeProblem(size_t horizon);

  // Returns the P and Q matrices for the problem statement.
  //   cost = 0.5 X.T P X + q.T X
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P(size_t horizon);
  const Eigen::Matrix<double, Eigen::Dynamic, 1> q(
      size_t horizon, Eigen::Matrix<double, 2, 1> X_initial,
      Eigen::Matrix<double, 2, 1> X_final);

 private:
  const Eigen::Matrix<double, Eigen::Dynamic, 1> accel_q(size_t horizon);

  const Eigen::Matrix<double, 2, 2> Af(size_t horizon);
  const Eigen::Matrix<double, 2, Eigen::Dynamic> Bf(size_t horizon);
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Pi(
      size_t horizon);

  // These functions are used in the constructor to build up the matrices below.
  Eigen::Matrix<double, Eigen::Dynamic, 2> MakeAs();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MakeBs();
  Eigen::Matrix<double, Eigen::Dynamic, 1> Makem();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MakeM();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MakeW();
  Eigen::Matrix<double, Eigen::Dynamic, 1> Makew();
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> MakePi();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MakeP();

  const StateFeedbackPlant<2, 1, 1> plant_;
  const size_t horizon_;

  const Eigen::DiagonalMatrix<double, 2> Q_final_;

  const Eigen::Matrix<double, Eigen::Dynamic, 2> As_;
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Bs_;
  const Eigen::Matrix<double, Eigen::Dynamic, 1> m_;
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M_;

  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> W_;
  const Eigen::Matrix<double, Eigen::Dynamic, 1> w_;
  const Eigen::DiagonalMatrix<double, Eigen::Dynamic> Pi_;

  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> WM_;
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Wmpw_;
};

}  // namespace catapult
}  // namespace control_loops
}  // namespace frc971