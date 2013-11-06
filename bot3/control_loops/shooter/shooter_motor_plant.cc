#include "frc971/control_loops/shooter/shooter_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<1, 1, 1> MakeShooterPlantCoefficients() {
  Eigen::Matrix<double, 1, 1> A;
  A << 0.989057756738;
  Eigen::Matrix<double, 1, 1> B;
  B << 0.533205953514;
  Eigen::Matrix<double, 1, 1> C;
  C << 1;
  Eigen::Matrix<double, 1, 1> D;
  D << 0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max << 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min << -12.0;
  return StateFeedbackPlantCoefficients<1, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<1, 1, 1> MakeShooterController() {
  Eigen::Matrix<double, 1, 1> L;
  L << 0.539057756738;
  Eigen::Matrix<double, 1, 1> K;
  K << 0.354567977894;
  return StateFeedbackController<1, 1, 1>(L, K, MakeShooterPlantCoefficients());
}

StateFeedbackPlant<1, 1, 1> MakeShooterPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<1, 1, 1> *> plants(1);
  plants[0] = new StateFeedbackPlantCoefficients<1, 1, 1>(MakeShooterPlantCoefficients());
  return StateFeedbackPlant<1, 1, 1>(plants);
}

StateFeedbackLoop<1, 1, 1> MakeShooterLoop() {
  ::std::vector<StateFeedbackController<1, 1, 1> *> controllers(1);
  controllers[0] = new StateFeedbackController<1, 1, 1>(MakeShooterController());
  return StateFeedbackLoop<1, 1, 1>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
