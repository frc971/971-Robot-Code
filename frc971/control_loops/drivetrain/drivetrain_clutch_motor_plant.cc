#include "frc971/control_loops/drivetrain/drivetrain_clutch_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeClutchDrivetrainPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00876671940282, 0.0, 0.000204905465153, 0.0, 0.764245148008, 0.0, 0.0373841350548, 0.0, 0.000204905465153, 1.0, 0.00876671940282, 0.0, 0.0373841350548, 0.0, 0.764245148008;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000157874070659, -2.62302512161e-05, 0.0301793267864, -0.00478559834045, -2.62302512161e-05, 0.000157874070659, -0.00478559834045, 0.0301793267864;
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

StateFeedbackController<4, 2, 2> MakeClutchDrivetrainController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.60424514801, 0.0373841350548, 53.4463554671, 4.58647914599, 0.0373841350548, 1.60424514801, 4.58647914599, 53.4463554671;
  Eigen::Matrix<double, 2, 4> K;
  K << 292.330461448, 10.4890095334, -85.5980253252, -0.517234397951, -58.0206391358, -1.5636023242, 153.384904309, 5.5616531565;
  return StateFeedbackController<4, 2, 2>(L, K, MakeClutchDrivetrainPlantCoefficients());
}

StateFeedbackPlant<4, 2, 2> MakeClutchDrivetrainPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<4, 2, 2> *> plants(1);
  plants[0] = new StateFeedbackPlantCoefficients<4, 2, 2>(MakeClutchDrivetrainPlantCoefficients());
  return StateFeedbackPlant<4, 2, 2>(plants);
}

StateFeedbackLoop<4, 2, 2> MakeClutchDrivetrainLoop() {
  ::std::vector<StateFeedbackController<4, 2, 2> *> controllers(1);
  controllers[0] = new StateFeedbackController<4, 2, 2>(MakeClutchDrivetrainController());
  return StateFeedbackLoop<4, 2, 2>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
