#include "frc971/control_loops/catapult/catapult_controller.h"

namespace frc971::control_loops::catapult {

CatapultController::CatapultController(StateFeedbackPlant<2, 1, 1> plant,
                                       size_t horizon)
    : generator_(std::move(plant), horizon) {
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

  double u;
  size_t solution_number = 0;
  if (current_controller_ == 0u) {
    while (solution_number < problems_[current_controller_]->horizon() &&
           problems_[current_controller_]->U(solution_number) < 0.01) {
      u = problems_[current_controller_]->U(solution_number);
      ++solution_number;
    }
  }
  u = problems_[current_controller_]->U(solution_number);

  if (current_controller_ + 1u + solution_number < problems_.size()) {
    problems_[current_controller_ + solution_number + 1]->WarmStart(
        *problems_[current_controller_]);
  }
  current_controller_ += 1u + solution_number;
  return u;
}

}  // namespace frc971::control_loops::catapult