#ifndef Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_CATAPULT_CATAPULT_H_
#define Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_CATAPULT_CATAPULT_H_

#include "Eigen/Dense"
#include "frc971/control_loops/state_feedback_loop.h"
#include "glog/logging.h"
#include "osqp++.h"

namespace y2022 {
namespace control_loops {
namespace superstructure {
namespace catapult {

// MPC problem for a specified horizon.  This contains all the state for the
// solver, setters to modify the current and target state, and a way to fetch
// the solution.
class MPCProblem {
 public:
  MPCProblem(size_t horizon,
             Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P,
             Eigen::Matrix<double, Eigen::Dynamic, 1> accel_q,
             Eigen::Matrix<double, 2, 2> Af,
             Eigen::Matrix<double, Eigen::Dynamic, 2> final_q);

  MPCProblem(MPCProblem const &) = delete;
  void operator=(MPCProblem const &x) = delete;

  // Sets the current and final state.  This keeps the problem in tact and
  // doesn't recreate it, so it will be fast.
  void SetState(Eigen::Matrix<double, 2, 1> X_initial,
                Eigen::Matrix<double, 2, 1> X_final);

  // Solves our problem.
  bool Solve();

  double solve_time() const { return solve_time_; }

  // Returns the solution that the solver found when Solve was last called.
  double U(size_t i) const { return solver_.primal_solution()(i); }

  // Returns the number of U's to be solved.
  size_t horizon() const { return horizon_; }

  // Warm starts the optimizer with the provided solution to make it solve
  // faster.
  void WarmStart(const MPCProblem &p);

 private:
  // The number of u's to solve for.
  const size_t horizon_;

  // The problem statement variables needed by SetState to update q.
  const Eigen::Matrix<double, Eigen::Dynamic, 1> accel_q_;
  const Eigen::Matrix<double, 2, 2> Af_;
  const Eigen::Matrix<double, Eigen::Dynamic, 2> final_q_;

  Eigen::Matrix<double, 2, 1> X_initial_;
  Eigen::Matrix<double, 2, 1> X_final_;

  // Solver state.
  osqp::OsqpInstance instance_;
  osqp::OsqpSolver solver_;
  osqp::OsqpSettings settings_;

  double solve_time_ = 0;
};

// Decently efficient problem generator for multiple horizons given a max
// horizon to solve for.
//
// The math is documented in mpc.tex
class CatapultProblemGenerator {
 public:
  // Builds a problem generator for the specified max horizon and caches a lot
  // of the state.
  CatapultProblemGenerator(size_t horizon);

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
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022

#endif  // Y2022_CONTROL_LOOPS_SUPERSTRUCTURE_CATAPULT_CATAPULT_H_
