#include "frc971/control_loops/angle_adjust/angle_adjust_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<3, 1, 1> MakeAngleAdjustPlantCoefficients() {
  Eigen::Matrix<double, 3, 3> A;
  A << 1.0, 0.00844804908295, 0.000186726546509, 0.0, 0.706562970689, 0.0353055515475, 0.0, 0.0, 1.0;
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

StateFeedbackController<3, 1, 1> MakeAngleAdjustController() {
  Eigen::Matrix<double, 3, 1> L;
  L << 1.82656297069, 80.440440048, 562.416026082;
  Eigen::Matrix<double, 1, 3> K;
  K << 171.984283883, 5.25098015082, 1.01656297069;
  return StateFeedbackController<3, 1, 1>(L, K, MakeAngleAdjustPlantCoefficients());
}

StateFeedbackPlant<3, 1, 1> MakeAngleAdjustPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<3, 1, 1> *> plants(1);
  plants[0] = new StateFeedbackPlantCoefficients<3, 1, 1>(MakeAngleAdjustPlantCoefficients());
  return StateFeedbackPlant<3, 1, 1>(plants);
}

StateFeedbackLoop<3, 1, 1> MakeAngleAdjustLoop() {
  ::std::vector<StateFeedbackController<3, 1, 1> *> controllers(1);
  controllers[0] = new StateFeedbackController<3, 1, 1>(MakeAngleAdjustController());
  return StateFeedbackLoop<3, 1, 1>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
