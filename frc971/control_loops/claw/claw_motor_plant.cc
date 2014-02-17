#include "frc971/control_loops/claw/claw_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeClawPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.0, 0.00904786878843, 0.0, 0.0, 1.0, 0.0, 0.00904786878843, 0.0, 0.0, 0.815818233346, 0.0, 0.0, 0.0, 0.0, 0.815818233346;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000326582411818, 0.0, 0.0, 0.000326582411818, 0.0631746179893, 0.0, 0.0, 0.0631746179893;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 1, 1, 0, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 24.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -24.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<4, 2, 2> MakeClawController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.71581823335, 5.38760974287e-16, -1.71581823335, 1.71581823335, 64.8264890043, 2.03572300346e-14, -64.8264890043, 64.8264890043;
  Eigen::Matrix<double, 2, 4> K;
  K << 146.100132128, 0.0, 6.7282816813, 0.0, 0.0, 275.346049928, 0.0, 12.0408756114;
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
