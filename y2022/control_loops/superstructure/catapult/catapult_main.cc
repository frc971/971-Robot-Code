#include "aos/init.h"
#include "aos/realtime.h"
#include "aos/time/time.h"
#include "y2022/control_loops/superstructure/catapult/catapult.h"
#include "y2022/control_loops/superstructure/catapult/catapult_plant.h"

namespace y2022 {
namespace control_loops {
namespace superstructure {
namespace catapult {
namespace chrono = std::chrono;

void OSQPSolve() {
  Eigen::Matrix<double, 2, 1> X_initial(0.0, 0.0);
  Eigen::Matrix<double, 2, 1> X_final(2.0, 25.0);

  LOG(INFO) << "Starting a dynamic OSQP solve";
  CatapultProblemGenerator g(35);
  const StateFeedbackPlant<2, 1, 1> plant = MakeCatapultPlant();

  constexpr int kHorizon = 35;

  // TODO(austin): This is a good unit test!  Make sure computing the problem
  // different ways comes out the same.
  {
    CatapultProblemGenerator g2(10);
    constexpr int kTestHorizon = 10;
    CHECK(g2.P(kTestHorizon) == g.P(kTestHorizon))
        << g2.P(kTestHorizon) - g.P(kTestHorizon);
    CHECK(g2.q(kTestHorizon, X_initial, X_final) ==
          g.q(kTestHorizon, X_initial, X_final))
        << g2.q(kTestHorizon, X_initial, X_final) -
               g.q(kTestHorizon, X_initial, X_final);
  }

  osqp::OsqpInstance instance;
  instance.objective_matrix = g.P(kHorizon).sparseView();

  instance.objective_vector = g.q(kHorizon, X_initial, X_final);

  instance.constraint_matrix =
      Eigen::SparseMatrix<double, Eigen::ColMajor, osqp::c_int>(kHorizon,
                                                                kHorizon);
  instance.constraint_matrix.setIdentity();

  instance.lower_bounds =
      Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(kHorizon, 1);
  instance.upper_bounds =
      Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(kHorizon, 1) * 12.0;

  osqp::OsqpSolver solver;
  osqp::OsqpSettings settings;
  // Edit settings if appropriate.
  auto status = solver.Init(instance, settings);
  CHECK(status.ok()) << status;

  aos::LockAllMemory();
  aos::ExpandStackSize();
  aos::SetCurrentThreadRealtimePriority(60);

  for (int i = 0; i < 10; ++i) {
    const aos::monotonic_clock::time_point start_time =
        aos::monotonic_clock::now();
    osqp::OsqpExitCode exit_code = solver.Solve();
    const aos::monotonic_clock::time_point end_time =
        aos::monotonic_clock::now();
    LOG(INFO) << "OSQP solved in "
              << chrono::duration<double>(end_time - start_time).count();
    CHECK(exit_code == osqp::OsqpExitCode::kOptimal);
  }

  double optimal_objective = solver.objective_value();
  Eigen::Matrix<double, Eigen::Dynamic, 1> optimal_solution =
      solver.primal_solution();

  LOG(INFO) << "Cost: " << optimal_objective;
  LOG(INFO) << "U: " << optimal_solution;

  std::vector<std::unique_ptr<MPCProblem>> problems;
  for (size_t i = g.horizon(); i > 0; --i) {
    LOG(INFO) << "Made problem " << i;
    problems.emplace_back(g.MakeProblem(i));
  }

  std::unique_ptr<MPCProblem> p = g.MakeProblem(kHorizon);

  p->SetState(X_initial, X_final);
  p->Solve();
  p->Solve();
  p->Solve();
  p->Solve();

  Eigen::Vector2d X = X_initial;
  for (size_t i = 0; i < g.horizon(); ++i) {
    problems[i]->SetState(X, X_final);
    if (i != 0) {
      problems[i]->WarmStart(*problems[i - 1]);
    }

    problems[i]->Solve();
    X = plant.A() * X + plant.B() * problems[i]->U(0);

    LOG(INFO) << "Dynamic u(" << i << "): " << problems[i]->U(0) << " -> "
              << X.transpose();
  }

  aos::UnsetCurrentThreadRealtimePriority();
}

int Main(int /*argc*/, char ** /*argv*/) {
  OSQPSolve();

  gflags::ShutDownCommandLineFlags();
  return 0;
}

}  // namespace catapult
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  return y2022::control_loops::superstructure::catapult::Main(argc, argv);
}
