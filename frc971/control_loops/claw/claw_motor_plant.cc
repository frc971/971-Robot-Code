#include "frc971/control_loops/claw/claw_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeClawPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.0, 0.00909331035298, 0.0, 0.0, 1.0, -6.04514323962e-05, 0.00903285892058, 0.0, 0.0, 0.824315642255, 0.0, 0.0, 0.0, -0.0112975266368, 0.813018115618;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000352527133889, 0.0, -0.000352527133889, 0.000376031064118, 0.0683072794628, 0.0, -0.0683072794628, 0.0726998350615;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 1, 1, 0, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<4, 2, 2> MakeClawController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.72431564225, 1.53329341668e-19, -1.72431564225, 1.71301811562, 65.9456997026, -0, -65.9456997026, 64.4642687194;
  Eigen::Matrix<double, 2, 4> K;
  K << 106.138028875, 0.0, 4.20012492658, 0.0, 99.5038407367, 99.7251230882, 3.77683310096, 3.78980738032;
  return StateFeedbackController<4, 2, 2>(L, K, MakeClawPlantCoefficients());
}

StateFeedbackPlant<4, 2, 2> MakeClawPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<4, 2, 2> *> plants(1);
  plants[0] = new StateFeedbackPlantCoefficients<4, 2, 2>(MakeClawPlantCoefficients());
  return StateFeedbackPlant<4, 2, 2>(plants);
}

StateFeedbackLoop<4, 2, 2> MakeClawLoop() {
  ::std::vector<StateFeedbackController<4, 2, 2> *> controllers(1);
  controllers[0] = new StateFeedbackController<4, 2, 2>(MakeClawController());
  return StateFeedbackLoop<4, 2, 2>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
