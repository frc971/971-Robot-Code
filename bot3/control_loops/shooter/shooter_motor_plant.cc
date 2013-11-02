#include "bot3/control_loops/shooter/shooter_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeShooterPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00994518845675, 0.0, 0.989057756738;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.00267091861198, 0.533205953514;
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

StateFeedbackController<2, 1, 1> MakeShooterController() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.08905775674, 29.7111780621;
  Eigen::Matrix<double, 1, 2> K;
  K << 1.42534042426, 0.758151303088;
  return StateFeedbackController<2, 1, 1>(L, K, MakeShooterPlantCoefficients());
}

StateFeedbackPlant<2, 1, 1> MakeShooterPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<2, 1, 1> *> plants(1);
  plants[0] = new StateFeedbackPlantCoefficients<2, 1, 1>(MakeShooterPlantCoefficients());
  return StateFeedbackPlant<2, 1, 1>(plants);
}

StateFeedbackLoop<2, 1, 1> MakeShooterLoop() {
  ::std::vector<StateFeedbackController<2, 1, 1> *> controllers(1);
  controllers[0] = new StateFeedbackController<2, 1, 1>(MakeShooterController());
  return StateFeedbackLoop<2, 1, 1>(controllers);
}

}  // namespace control_loops
}  // namespace bot3
