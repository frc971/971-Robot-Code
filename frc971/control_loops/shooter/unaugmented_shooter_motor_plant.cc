#include "frc971/control_loops/shooter/unaugmented_shooter_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeRawShooterPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00988697090637, 0.0, 0.977479674375;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.000120553991591, 0.0240196135246;
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

StateFeedbackController<2, 1, 1> MakeRawShooterController() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.87747967438, 87.0117404538;
  Eigen::Matrix<double, 1, 2> K;
  K << 343.469306513, 26.4814035344;
  return StateFeedbackController<2, 1, 1>(L, K, MakeRawShooterPlantCoefficients());
}

StateFeedbackPlant<2, 1, 1> MakeRawShooterPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<2, 1, 1> *> plants(1);
  plants[0] = new StateFeedbackPlantCoefficients<2, 1, 1>(MakeRawShooterPlantCoefficients());
  return StateFeedbackPlant<2, 1, 1>(plants);
}

StateFeedbackLoop<2, 1, 1> MakeRawShooterLoop() {
  ::std::vector<StateFeedbackController<2, 1, 1> *> controllers(1);
  controllers[0] = new StateFeedbackController<2, 1, 1>(MakeRawShooterController());
  return StateFeedbackLoop<2, 1, 1>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
