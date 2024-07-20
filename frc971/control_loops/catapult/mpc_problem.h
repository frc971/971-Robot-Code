#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/realtime.h"
#include "aos/time/time.h"
#include "osqp++.h"
#include "osqp.h"

namespace frc971 {
namespace control_loops {
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

  Eigen::Matrix<double, Eigen::Dynamic, 1> objective_vector_;

  // Solver state.
  osqp::OsqpInstance instance_;
  osqp::OsqpSolver solver_;
  osqp::OsqpSettings settings_;

  double solve_time_ = 0;
};

}  // namespace catapult
}  // namespace control_loops
}  // namespace frc971