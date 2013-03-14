#include "frc971/control_loops/wrist/wrist_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeWristPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00904786878843, 0.0, 0.815818233346;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.000326582411818, 0.0631746179893;
  Eigen::Matrix<double, 1, 2> C;
  C << 1, 0;
  Eigen::Matrix<double, 1, 1> D;
  D << 0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max << 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min << -12.0;
  return StateFeedbackPlantCoefficients<2, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<2, 1, 1> MakeWristController() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.71581823335, 64.8264890043;
  Eigen::Matrix<double, 1, 2> K;
  K << 119.668313646, 7.2297495639;
  return StateFeedbackController<2, 1, 1>(L, K, MakeWristPlantCoefficients());
}

StateFeedbackPlant<2, 1, 1> MakeWristPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<2, 1, 1> *> plants(1);
  plants[0] = new StateFeedbackPlantCoefficients<2, 1, 1>(MakeWristPlantCoefficients());
  return StateFeedbackPlant<2, 1, 1>(plants);
}

StateFeedbackLoop<2, 1, 1> MakeWristLoop() {
  ::std::vector<StateFeedbackController<2, 1, 1> *> controllers(1);
  controllers[0] = new StateFeedbackController<2, 1, 1>(MakeWristController());
  return StateFeedbackLoop<2, 1, 1>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
