#include "bot3/control_loops/drivetrain/drivetrain_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.0095410093, 0.0, -0.0000167223, 0.0, 0.9096302600, 0.0, -0.0032396985, 0.0, -0.0000167223, 1.0, 0.0095410093, 0.0, -0.0032396985, 0.0, 0.9096302600;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.0000628338, 0.0000022892, 0.0123712263, 0.0004435007, 0.0000022892, 0.0000628338, 0.0004435007, 0.0123712263;
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
  L << 1.7496302600, -0.0032396985, 72.1296388532, -0.4369906587, -0.0032396985, 1.7496302600, -0.4369906586, 72.1296388532;
  Eigen::Matrix<double, 2, 4> K;
  K << 242.8102455120, 19.7898401032, -8.7045950610, -0.9720464423, -8.7045950610, -0.9720464423, 242.8102455120, 19.7898401032;
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
}  // namespace bot3
