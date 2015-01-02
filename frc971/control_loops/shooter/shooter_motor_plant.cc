#include "frc971/control_loops/shooter/shooter_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<3, 1, 1> MakeSprungShooterPlantCoefficients() {
  Eigen::Matrix<double, 3, 3> A;
  A << 0.999391114909, 0.00811316740387, 7.59766686183e-05, -0.113584343654, 0.64780421498, 0.0141730519709, 0.0, 0.0, 1.0;
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

StateFeedbackPlantCoefficients<3, 1, 1> MakeShooterPlantCoefficients() {
  Eigen::Matrix<double, 3, 3> A;
  A << 1.0, 0.00811505488455, 7.59852687598e-05, 0.0, 0.648331305446, 0.0141763492481, 0.0, 0.0, 1.0;
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

StateFeedbackController<3, 1, 1> MakeSprungShooterController() {
  Eigen::Matrix<double, 3, 1> L;
  L << 1.64719532989, 57.0572680832, 636.74290365;
  Eigen::Matrix<double, 1, 3> K;
  K << 450.571849185, 11.8404918938, 0.997195329889;
  return StateFeedbackController<3, 1, 1>(L, K, MakeSprungShooterPlantCoefficients());
}

StateFeedbackController<3, 1, 1> MakeShooterController() {
  Eigen::Matrix<double, 3, 1> L;
  L << 1.64833130545, 57.2417604572, 636.668851906;
  Eigen::Matrix<double, 1, 3> K;
  K << 349.173113146, 8.65077618169, 0.848331305446;
  return StateFeedbackController<3, 1, 1>(L, K, MakeShooterPlantCoefficients());
}

StateFeedbackPlant<3, 1, 1> MakeShooterPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<3, 1, 1> *> plants(2);
  plants[0] = new StateFeedbackPlantCoefficients<3, 1, 1>(MakeSprungShooterPlantCoefficients());
  plants[1] = new StateFeedbackPlantCoefficients<3, 1, 1>(MakeShooterPlantCoefficients());
  return StateFeedbackPlant<3, 1, 1>(plants);
}

StateFeedbackLoop<3, 1, 1> MakeShooterLoop() {
  ::std::vector<StateFeedbackController<3, 1, 1> *> controllers(2);
  controllers[0] = new StateFeedbackController<3, 1, 1>(MakeSprungShooterController());
  controllers[1] = new StateFeedbackController<3, 1, 1>(MakeShooterController());
  return StateFeedbackLoop<3, 1, 1>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
