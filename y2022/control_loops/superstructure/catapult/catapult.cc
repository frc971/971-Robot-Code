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
  objective_vector_ =
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
    size_t horizon, Eigen::Matrix<double, 2, 1> X_initial,
    Eigen::Matrix<double, 2, 1> X_final) {
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

CatapultController::CatapultController(size_t horizon) : generator_(horizon) {
  problems_.reserve(generator_.horizon());
  for (size_t i = generator_.horizon(); i > 0; --i) {
    problems_.emplace_back(generator_.MakeProblem(i));
  }

  Reset();
}

void CatapultController::Reset() {
  current_controller_ = 0;
  solve_time_ = 0.0;
}

void CatapultController::SetState(Eigen::Matrix<double, 2, 1> X_initial,
                                  Eigen::Matrix<double, 2, 1> X_final) {
  if (current_controller_ >= problems_.size()) {
    return;
  }
  problems_[current_controller_]->SetState(X_initial, X_final);
}

bool CatapultController::Solve() {
  if (current_controller_ >= problems_.size()) {
    return true;
  }
  const bool result = problems_[current_controller_]->Solve();
  solve_time_ = problems_[current_controller_]->solve_time();
  return result;
}

std::optional<double> CatapultController::Next() {
  if (current_controller_ >= problems_.size()) {
    return std::nullopt;
  }

  const double u = problems_[current_controller_]->U(0);

  if (current_controller_ + 1u < problems_.size()) {
    problems_[current_controller_ + 1]->WarmStart(
        *problems_[current_controller_]);
  }
  ++current_controller_;
  return u;
}

const flatbuffers::Offset<
    frc971::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>
Catapult::Iterate(const Goal *unsafe_goal, const Position *position,
                  double battery_voltage, double *catapult_voltage, bool fire,
                  flatbuffers::FlatBufferBuilder *fbb) {
  const frc971::control_loops::StaticZeroingSingleDOFProfiledSubsystemGoal
      *catapult_goal = unsafe_goal != nullptr && unsafe_goal->has_catapult()
                           ? (unsafe_goal->catapult()->return_position())
                           : nullptr;

  const bool catapult_disabled = catapult_.Correct(
      catapult_goal, position->catapult(), catapult_voltage == nullptr);

  if (catapult_disabled) {
    catapult_state_ = CatapultState::PROFILE;
  } else if (catapult_.running() && unsafe_goal &&
             unsafe_goal->has_catapult() && fire && !last_firing_) {
    catapult_state_ = CatapultState::FIRING;
  }

  if (catapult_.running() && unsafe_goal && unsafe_goal->has_catapult()) {
    last_firing_ = fire;
  }

  use_profile_ = true;

  switch (catapult_state_) {
    case CatapultState::FIRING: {
      // Select the ball controller.  We should only be firing if we have a
      // ball, or at least should only care about the shot accuracy.
      catapult_.set_controller_index(0);
      // Ok, so we've now corrected.  Next step is to run the MPC.
      //
      // Since there is a unit delay between when we ask for a U and the
      // hardware applies it, we need to run the optimizer for the position at
      // the *next* control loop cycle.

      const Eigen::Vector3d next_X =
          catapult_.controller().plant().A() * catapult_.estimated_state() +
          catapult_.controller().plant().B() *
              catapult_.controller().observer().last_U();

      catapult_mpc_.SetState(
          next_X.block<2, 1>(0, 0),
          Eigen::Vector2d(unsafe_goal->catapult()->shot_position(),
                          unsafe_goal->catapult()->shot_velocity()));

      const bool solved = catapult_mpc_.Solve();

      if (solved || catapult_mpc_.started()) {
        std::optional<double> solution = catapult_mpc_.Next();

        if (!solution.has_value()) {
          CHECK_NOTNULL(catapult_voltage);
          *catapult_voltage = 0.0;
          if (catapult_mpc_.started()) {
            ++shot_count_;
            // Finished the catapult, time to fire.
            catapult_state_ = CatapultState::RESETTING;
          }
        } else {
          // TODO(austin): Voltage error?
          CHECK_NOTNULL(catapult_voltage);
          *catapult_voltage = std::max(
              0.0, std::min(12.0, (*solution - 0.0 * next_X(2, 0)) * 12.0 /
                                      std::max(battery_voltage, 8.0)));
          use_profile_ = false;
        }
      } else {
        if (unsafe_goal && unsafe_goal->has_catapult() && !fire) {
          // Eh, didn't manage to solve before it was time to fire.  Give up.
          catapult_state_ = CatapultState::PROFILE;
        }
      }

      if (!use_profile_) {
        catapult_.ForceGoal(catapult_.estimated_position(),
                            catapult_.estimated_velocity());
      }
    }
      if (catapult_state_ != CatapultState::RESETTING) {
        break;
      } else {
        [[fallthrough]];
      }

    case CatapultState::RESETTING:
      if (catapult_.controller().R(1, 0) > 7.0) {
        catapult_.AdjustProfile(7.0, 2000.0);
      } else if (catapult_.controller().R(1, 0) > 0.0) {
        catapult_.AdjustProfile(7.0, 1000.0);
      } else {
        catapult_state_ = CatapultState::PROFILE;
      }
      [[fallthrough]];

    case CatapultState::PROFILE:
      break;
  }

  if (use_profile_) {
    if (catapult_state_ != CatapultState::FIRING) {
      catapult_mpc_.Reset();
    }
    // Select the controller designed for when we have no ball.
    catapult_.set_controller_index(1);

    const double output_voltage = catapult_.UpdateController(catapult_disabled);
    if (catapult_voltage != nullptr) {
      *catapult_voltage = output_voltage;
    }
  }

  catapult_.UpdateObserver(catapult_voltage != nullptr
                               ? (*catapult_voltage * battery_voltage / 12.0)
                               : 0.0);

  return catapult_.MakeStatus(fbb);
}

}  // namespace catapult
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2022
