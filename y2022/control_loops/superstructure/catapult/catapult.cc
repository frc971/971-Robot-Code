#include "y2022/control_loops/superstructure/catapult/catapult.h"

#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "aos/realtime.h"
#include "aos/time/time.h"
#include "glog/logging.h"
#include "osqp++.h"
#include "osqp.h"
#include "y2022/control_loops/superstructure/catapult/catapult_plant.h"

namespace y2022 {
namespace control_loops {
namespace superstructure {
namespace catapult {
namespace chrono = std::chrono;

namespace {
osqp::OsqpInstance MakeInstance(
    size_t horizon, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P) {
  osqp::OsqpInstance instance;
  instance.objective_matrix = P.cast<c_float>().sparseView();

  instance.constraint_matrix =
      Eigen::SparseMatrix<c_float, Eigen::ColMajor, osqp::c_int>(horizon,
                                                                 horizon);
  instance.constraint_matrix.setIdentity();

  instance.lower_bounds =
      Eigen::Matrix<c_float, Eigen::Dynamic, 1>::Zero(horizon, 1);
  instance.upper_bounds =
      Eigen::Matrix<c_float, Eigen::Dynamic, 1>::Ones(horizon, 1) * 12.0;
  return instance;
}
}  // namespace

MPCProblem::MPCProblem(size_t horizon,
                       Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> P,
                       Eigen::Matrix<double, Eigen::Dynamic, 1> accel_q,
                       Eigen::Matrix<double, 2, 2> Af,
                       Eigen::Matrix<double, Eigen::Dynamic, 2> final_q)
    : horizon_(horizon),
      accel_q_(std::move(accel_q.cast<c_float>())),
      Af_(std::move(Af.cast<c_float>())),
      final_q_(std::move(final_q.cast<c_float>())),
      instance_(MakeInstance(horizon, std::move(P))) {
  instance_.objective_vector =
      Eigen::Matrix<c_float, Eigen::Dynamic, 1>(horizon, 1);
  settings_.max_iter = 25;
  settings_.check_termination = 25;
  settings_.warm_start = 0;
  auto status = solver_.Init(instance_, settings_);
  CHECK(status.ok()) << status;
}

void MPCProblem::SetState(Eigen::Matrix<c_float, 2, 1> X_initial,
                          Eigen::Matrix<c_float, 2, 1> X_final) {
  X_initial_ = X_initial;
  X_final_ = X_final;
  instance_.objective_vector =
      X_initial(1, 0) * accel_q_ + final_q_ * (Af_ * X_initial - X_final);

  auto status = solver_.SetObjectiveVector(instance_.objective_vector);
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

CatapultProblemGenerator::CatapultProblemGenerator(size_t horizon)
    : plant_(MakeCatapultPlant()),
      horizon_(horizon),
      Q_final_(
          (Eigen::DiagonalMatrix<double, 2>().diagonal() << 10000.0, 10000.0)
              .finished()),
      As_(MakeAs()),
      Bs_(MakeBs()),
      m_(Makem()),
      M_(MakeM()),
      W_(MakeW()),
      w_(Makew()),
      Pi_(MakePi()),
      WM_(W_ * M_),
      Wmpw_(W_ * m_ + w_) {}

std::unique_ptr<MPCProblem> CatapultProblemGenerator::MakeProblem(
    size_t horizon) {
  return std::make_unique<MPCProblem>(
      horizon, P(horizon), accel_q(horizon), Af(horizon),
      (2.0 * Q_final_ * Bf(horizon)).transpose());
}

const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
CatapultProblemGenerator::P(size_t horizon) {
  CHECK_GT(horizon, 0u);
  CHECK_LE(horizon, horizon_);
  return 2.0 * (WM_.block(0, 0, horizon, horizon).transpose() * Pi(horizon) *
                    WM_.block(0, 0, horizon, horizon) +
                Bf(horizon).transpose() * Q_final_ * Bf(horizon));
}

const Eigen::Matrix<double, Eigen::Dynamic, 1> CatapultProblemGenerator::q(
    size_t horizon, Eigen::Matrix<c_float, 2, 1> X_initial,
    Eigen::Matrix<c_float, 2, 1> X_final) {
  CHECK_GT(horizon, 0u);
  CHECK_LE(horizon, horizon_);
  return 2.0 * X_initial(1, 0) * accel_q(horizon) +
         2.0 * ((Af(horizon) * X_initial - X_final).transpose() * Q_final_ *
                Bf(horizon))
                   .transpose();
}

const Eigen::Matrix<double, Eigen::Dynamic, 1>
CatapultProblemGenerator::accel_q(size_t horizon) {
  return 2.0 * ((Wmpw_.block(0, 0, horizon, 1)).transpose() * Pi(horizon) *
                WM_.block(0, 0, horizon, horizon))
                   .transpose();
}

const Eigen::Matrix<double, 2, 2> CatapultProblemGenerator::Af(size_t horizon) {
  CHECK_GT(horizon, 0u);
  CHECK_LE(horizon, horizon_);
  return As_.block<2, 2>(2 * (horizon - 1), 0);
}

const Eigen::Matrix<double, 2, Eigen::Dynamic> CatapultProblemGenerator::Bf(
    size_t horizon) {
  CHECK_GT(horizon, 0u);
  CHECK_LE(horizon, horizon_);
  return Bs_.block(2 * (horizon - 1), 0, 2, horizon);
}

const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
CatapultProblemGenerator::Pi(size_t horizon) {
  CHECK_GT(horizon, 0u);
  CHECK_LE(horizon, horizon_);
  return Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>(Pi_).block(
      horizon_ - horizon, horizon_ - horizon, horizon, horizon);
}

Eigen::Matrix<double, Eigen::Dynamic, 2> CatapultProblemGenerator::MakeAs() {
  Eigen::Matrix<double, Eigen::Dynamic, 2> As =
      Eigen::Matrix<double, Eigen::Dynamic, 2>::Zero(horizon_ * 2, 2);
  for (size_t i = 0; i < horizon_; ++i) {
    if (i == 0) {
      As.block<2, 2>(0, 0) = plant_.A();
    } else {
      As.block<2, 2>(i * 2, 0) = plant_.A() * As.block<2, 2>((i - 1) * 2, 0);
    }
  }
  return As;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
CatapultProblemGenerator::MakeBs() {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Bs =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(horizon_ * 2,
                                                                  horizon_);
  for (size_t i = 0; i < horizon_; ++i) {
    for (size_t j = 0; j < i + 1; ++j) {
      if (i == j) {
        Bs.block<2, 1>(i * 2, j) = plant_.B();
      } else {
        Bs.block<2, 1>(i * 2, j) =
            As_.block<2, 2>((i - j - 1) * 2, 0) * plant_.B();
      }
    }
  }
  return Bs;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> CatapultProblemGenerator::Makem() {
  Eigen::Matrix<double, Eigen::Dynamic, 1> m =
      Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(horizon_, 1);
  for (size_t i = 0; i < horizon_; ++i) {
    m(i, 0) = As_(1 + 2 * i, 1);
  }
  return m;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
CatapultProblemGenerator::MakeM() {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(horizon_,
                                                                  horizon_);
  for (size_t i = 0; i < horizon_; ++i) {
    for (size_t j = 0; j < horizon_; ++j) {
      M(i, j) = Bs_(2 * i + 1, j);
    }
  }
  return M;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
CatapultProblemGenerator::MakeW() {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> W =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(horizon_,
                                                                      horizon_);
  for (size_t i = 0; i < horizon_ - 1; ++i) {
    W(i + 1, i) = -1.0;
  }
  W /= std::chrono::duration<double>(plant_.dt()).count();
  return W;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> CatapultProblemGenerator::Makew() {
  Eigen::Matrix<double, Eigen::Dynamic, 1> w =
      Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(horizon_, 1);
  w(0, 0) = -1.0 / std::chrono::duration<double>(plant_.dt()).count();
  return w;
}

Eigen::DiagonalMatrix<double, Eigen::Dynamic>
CatapultProblemGenerator::MakePi() {
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> Pi(horizon_);
  for (size_t i = 0; i < horizon_; ++i) {
    Pi.diagonal()(i) =
        std::pow(0.01, 2.0) +
        std::pow(0.02 * std::max(0.0, (20 - ((int)horizon_ - (int)i)) / 20.),
                 2.0);
  }
  return Pi;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
CatapultProblemGenerator::MakeP() {
  return 2.0 * (M_.transpose() * W_.transpose() * Pi_ * W_ * M_ +
                Bf(horizon_).transpose() * Q_final_ * Bf(horizon_));
}

}  // namespace catapult
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022
