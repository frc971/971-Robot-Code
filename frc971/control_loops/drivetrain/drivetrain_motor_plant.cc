#include "frc971/control_loops/drivetrain/drivetrain_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00948696019317, 0.0, 3.55966215909e-06, 0.0, 0.899177606502, 0.0, 0.000686937184856, 0.0, 3.55966215909e-06, 1.0, 0.00948696019317, 0.0, 0.000686937184856, 0.0, 0.899177606502;
  Eigen::Matrix<double, 4, 2> B;
  B << 9.50723568824e-05, -6.59647588097e-07, 0.0186835844877, -0.000127297602107, -6.59647588097e-07, 9.50723568824e-05, -0.000127297602107, 0.0186835844877;
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
  L << 1.7391776065, 0.000686937184856, 70.7236123469, 0.0920942992696, 0.000686937184856, 1.7391776065, 0.0920942992696, 70.7236123469;
  Eigen::Matrix<double, 2, 4> K;
  K << 318.476158856, 20.8163224602, 2.1698861574, 0.178798182963, 2.1698861574, 0.178798182963, 318.476158856, 20.8163224602;
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
