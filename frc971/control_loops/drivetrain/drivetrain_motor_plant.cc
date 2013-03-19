#include "frc971/control_loops/drivetrain/drivetrain_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00931379160739, 0.0, 4.70184876909e-06, 0.0, 0.865971883056, 0.0, 0.000895808426591, 0.0, 4.70184876909e-06, 1.0, 0.00931379160739, 0.0, 0.000895808426591, 0.0, 0.865971883056;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000126707931029, -8.6819330098e-07, 0.0247482041615, -0.000165410440259, -8.6819330098e-07, 0.000126707931029, -0.000165410440259, 0.0247482041615;
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

StateFeedbackController<4, 2, 2> MakeDrivetrainController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.70597188306, 0.000895808426591, 66.3158545945, 0.117712892743, 0.000895808426591, 1.70597188306, 0.117712892743, 66.3158545945;
  Eigen::Matrix<double, 2, 4> K;
  K << 240.432225842, 14.3659115621, 1.60698530163, 0.13242189318, 1.60698530163, 0.13242189318, 240.432225842, 14.3659115621;
  return StateFeedbackController<4, 2, 2>(L, K, MakeDrivetrainPlantCoefficients());
}

StateFeedbackPlant<4, 2, 2> MakeDrivetrainPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<4, 2, 2> *> plants(1);
  plants[0] = new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainPlantCoefficients());
  return StateFeedbackPlant<4, 2, 2>(plants);
}

StateFeedbackLoop<4, 2, 2> MakeDrivetrainLoop() {
  ::std::vector<StateFeedbackController<4, 2, 2> *> controllers(1);
  controllers[0] = new StateFeedbackController<4, 2, 2>(MakeDrivetrainController());
  return StateFeedbackLoop<4, 2, 2>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
