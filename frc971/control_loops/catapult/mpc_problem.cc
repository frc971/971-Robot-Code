#include "frc971/control_loops/catapult/mpc_problem.h"

namespace frc971::control_loops::catapult {
namespace chrono = std::chrono;

namespace {
osqp::OsqpInstance MakeInstance(
    size_t horizon, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P) {
  osqp::OsqpInstance instance;
  instance.objective_matrix = P.sparseView();

  instance.constraint_matrix =
      Eigen::SparseMatrix<double, Eigen::ColMajor, osqp::c_int>(horizon,
                                                                horizon);
  instance.constraint_matrix.setIdentity();

  instance.lower_bounds =
      Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(horizon, 1);
  instance.upper_bounds =
      Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(horizon, 1) * 12.0;
  return instance;
}

}  // namespace
MPCProblem::MPCProblem(size_t horizon,
                       Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P,
                       Eigen::Matrix<double, Eigen::Dynamic, 1> accel_q,
                       Eigen::Matrix<double, 2, 2> Af,
                       Eigen::Matrix<double, Eigen::Dynamic, 2> final_q)
    : horizon_(horizon),
      accel_q_(std::move(accel_q)),
      Af_(std::move(Af)),
      final_q_(std::move(final_q)),
      instance_(MakeInstance(horizon, std::move(P))) {
  // Start with a representative problem.
  Eigen::Matrix<double, 2, 1> X_initial(0.0, 0.0);
  Eigen::Matrix<double, 2, 1> X_final(2.0, 25.0);

  objective_vector_ =
      X_initial(1, 0) * accel_q_ + final_q_ * (Af_ * X_initial - X_final);
  instance_.objective_vector = objective_vector_;
  settings_.max_iter = 25;
  settings_.check_termination = 5;
  settings_.warm_start = 1;
  // TODO(austin): Do we need this scaling thing?  It makes it not solve
  // sometimes... I'm pretty certain by giving it a decently formed problem to
  // initialize with, it will not try doing crazy things with the scaling
  // internally.
  settings_.scaling = 0;
  auto status = solver_.Init(instance_, settings_);
  CHECK(status.ok()) << status;
}

void MPCProblem::SetState(Eigen::Matrix<double, 2, 1> X_initial,
                          Eigen::Matrix<double, 2, 1> X_final) {
  X_initial_ = X_initial;
  X_final_ = X_final;
  // If we mark this noalias(), it won't re-allocate the vector each time.
  objective_vector_.noalias() =
      X_initial(1, 0) * accel_q_ + final_q_ * (Af_ * X_initial - X_final);

  auto status = solver_.SetObjectiveVector(objective_vector_);
  CHECK(status.ok()) << status;
}

bool MPCProblem::Solve() {
  const aos::monotonic_clock::time_point start_time =
      aos::monotonic_clock::now();
  osqp::OsqpExitCode exit_code = solver_.Solve();
  const aos::monotonic_clock::time_point end_time = aos::monotonic_clock::now();
  VLOG(1) << "OSQP solved in "
          << std::chrono::duration<double>(end_time - start_time).count();
  solve_time_ = std::chrono::duration<double>(end_time - start_time).count();
  // TODO(austin): Dump the exit codes out as an enum for logging.
  //
  // TODO(austin): The dual problem doesn't appear to be converging on all
  // problems.  Are we phrasing something wrong?

  // TODO(austin): Set a time limit so we can't run forever, and signal back
  // when we hit our limit.
  return exit_code == osqp::OsqpExitCode::kOptimal;
}

void MPCProblem::WarmStart(const MPCProblem &p) {
  CHECK_GE(p.horizon(), horizon())
      << ": Can only copy a bigger problem's solution into a smaller problem.";
  auto status = solver_.SetPrimalWarmStart(p.solver_.primal_solution().block(
      p.horizon() - horizon(), 0, horizon(), 1));
  CHECK(status.ok()) << status;
  status = solver_.SetDualWarmStart(p.solver_.dual_solution().block(
      p.horizon() - horizon(), 0, horizon(), 1));
  CHECK(status.ok()) << status;
}

}  // namespace frc971::control_loops::catapult