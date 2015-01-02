#include "frc971/control_loops/shooter/unaugmented_shooter_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeRawSprungShooterPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.999391114909, 0.00811316740387, -0.113584343654, 0.64780421498;
  Eigen::Matrix<double, 2, 1> B;
  B << 7.59766686183e-05, 0.0141730519709;
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

StateFeedbackPlantCoefficients<2, 1, 1> MakeRawShooterPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00811505488455, 0.0, 0.648331305446;
  Eigen::Matrix<double, 2, 1> B;
  B << 7.59852687598e-05, 0.0141763492481;
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

StateFeedbackController<2, 1, 1> MakeRawSprungShooterController() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.54719532989, 43.9345489758;
  Eigen::Matrix<double, 1, 2> K;
  K << 2126.06977433, 41.3223370936;
  return StateFeedbackController<2, 1, 1>(L, K, MakeRawSprungShooterPlantCoefficients());
}

StateFeedbackController<2, 1, 1> MakeRawShooterController() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.54833130545, 44.1155797675;
  Eigen::Matrix<double, 1, 2> K;
  K << 2133.83569145, 41.3499425476;
  return StateFeedbackController<2, 1, 1>(L, K, MakeRawShooterPlantCoefficients());
}

StateFeedbackPlant<2, 1, 1> MakeRawShooterPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<2, 1, 1> *> plants(2);
  plants[0] = new StateFeedbackPlantCoefficients<2, 1, 1>(MakeRawSprungShooterPlantCoefficients());
  plants[1] = new StateFeedbackPlantCoefficients<2, 1, 1>(MakeRawShooterPlantCoefficients());
  return StateFeedbackPlant<2, 1, 1>(plants);
}

StateFeedbackLoop<2, 1, 1> MakeRawShooterLoop() {
  ::std::vector<StateFeedbackController<2, 1, 1> *> controllers(2);
  controllers[0] = new StateFeedbackController<2, 1, 1>(MakeRawSprungShooterController());
  controllers[1] = new StateFeedbackController<2, 1, 1>(MakeRawShooterController());
  return StateFeedbackLoop<2, 1, 1>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
