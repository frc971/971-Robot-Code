#include "frc971/control_loops/wrists/top_wrist_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<3, 1, 1> MakeWristPlantCoefficients() {
  Eigen::Matrix<double, 3, 3> A;
  A << 1.0, 0.00904786878843, 0.000326582411818, 0.0, 0.815818233346, 0.0631746179893, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 3, 1> B;
  B << 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 1, 3> C;
  C << 1.0, 0.0, 0.0;
  Eigen::Matrix<double, 1, 1> D;
  D << 0.0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max << 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min << -12.0;
  return StateFeedbackPlantCoefficients<3, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<3, 1, 1> MakeWristController() {
  Eigen::Matrix<double, 3, 1> L;
  L << 1.81581823335, 78.6334534274, 142.868137351;
  Eigen::Matrix<double, 1, 3> K;
  K << 92.6004807973, 4.38063492858, 1.11581823335;
  return StateFeedbackController<3, 1, 1>(L, K, MakeWristPlantCoefficients());
}

StateFeedbackPlant<3, 1, 1> MakeWristPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<3, 1, 1> *> plants(1);
  plants[0] = new StateFeedbackPlantCoefficients<3, 1, 1>(MakeWristPlantCoefficients());
  return StateFeedbackPlant<3, 1, 1>(plants);
}

StateFeedbackLoop<3, 1, 1> MakeWristLoop() {
  ::std::vector<StateFeedbackController<3, 1, 1> *> controllers(1);
  controllers[0] = new StateFeedbackController<3, 1, 1>(MakeWristController());
  return StateFeedbackLoop<3, 1, 1>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
