#include "frc971/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDogDrivetrainPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00923787174605, 0.0, 0.000131162317098, 0.0, 0.851672729447, 0.0, 0.0248457251406, 0.0, 0.000131162317098, 1.0, 0.00923787174605, 0.0, 0.0248457251406, 0.0, 0.851672729447;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000126364935405, -2.17474127771e-05, 0.0245934537462, -0.00411955394149, -2.17474127771e-05, 0.000126364935405, -0.00411955394149, 0.0245934537462;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 0, 0, 1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<4, 2, 2> MakeDogDrivetrainController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.69167272945, 0.0248457251406, 64.4706646869, 3.2355304474, 0.0248457251406, 1.69167272945, 3.2355304474, 64.4706646869;
  Eigen::Matrix<double, 2, 4> K;
  K << 248.918529922, 14.4460993245, 41.6953764051, 3.43594323497, 41.6953764051, 3.43594323497, 248.918529922, 14.4460993245;
  return StateFeedbackController<4, 2, 2>(L, K, MakeDogDrivetrainPlantCoefficients());
}

StateFeedbackPlant<4, 2, 2> MakeDogDrivetrainPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<4, 2, 2> *> plants(1);
  plants[0] = new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDogDrivetrainPlantCoefficients());
  return StateFeedbackPlant<4, 2, 2>(plants);
}

StateFeedbackLoop<4, 2, 2> MakeDogDrivetrainLoop() {
  ::std::vector<StateFeedbackController<4, 2, 2> *> controllers(1);
  controllers[0] = new StateFeedbackController<4, 2, 2>(MakeDogDrivetrainController());
  return StateFeedbackLoop<4, 2, 2>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
