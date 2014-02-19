#include "frc971/control_loops/shooter/shooter_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<3, 1, 1> MakeShooterPlantCoefficients() {
  Eigen::Matrix<double, 3, 3> A;
  A << 0.998324052598, 0.0007783475087, 0.000278701304898, -0.181614418697, -0.000138907346386, 0.0302015298419, 0.0, 0.0, 1.0;
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

StateFeedbackController<3, 1, 1> MakeShooterController() {
  Eigen::Matrix<double, 3, 1> L;
  L << 0.998185145251, 11.8167175789, 298.617717297;
  Eigen::Matrix<double, 1, 3> K;
  K << 162.58140285, 6.68264124674, 0.198185145251;
  return StateFeedbackController<3, 1, 1>(L, K, MakeShooterPlantCoefficients());
}

StateFeedbackPlant<3, 1, 1> MakeShooterPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<3, 1, 1> *> plants(1);
  plants[0] = new StateFeedbackPlantCoefficients<3, 1, 1>(MakeShooterPlantCoefficients());
  return StateFeedbackPlant<3, 1, 1>(plants);
}

StateFeedbackLoop<3, 1, 1> MakeShooterLoop() {
  ::std::vector<StateFeedbackController<3, 1, 1> *> controllers(1);
  controllers[0] = new StateFeedbackController<3, 1, 1>(MakeShooterController());
  return StateFeedbackLoop<3, 1, 1>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
