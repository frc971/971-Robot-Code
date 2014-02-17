#include "frc971/control_loops/shooter/shooter_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<3, 1, 1> MakeShooterPlantCoefficients() {
  Eigen::Matrix<double, 3, 3> A;
  A << 1.0, 0.00988697090637, 0.000120553991591, 0.0, 0.977479674375, 0.0240196135246, 0.0, 0.0, 1.0;
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
  L << 1.97747967438, 101.419434198, 375.761249895;
  Eigen::Matrix<double, 1, 3> K;
  K << 243.5509628, 18.9166116502, 1.27747967438;
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
