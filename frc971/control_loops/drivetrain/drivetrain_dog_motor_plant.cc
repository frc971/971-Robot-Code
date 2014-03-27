#include "frc971/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00860955515291, 0.0, 0.000228184998733, 0.0, 0.735841675858, 0.0, 0.0410810558113, 0.0, 0.000228184998733, 1.0, 0.00860955515291, 0.0, 0.0410810558113, 0.0, 0.735841675858;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000272244648044, -4.46778919705e-05, 0.0517213538779, -0.00804353916233, -4.46778919705e-05, 0.000272244648044, -0.00804353916233, 0.0517213538779;
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
  L << 1.57584167586, 0.0410810558113, 50.0130674801, 4.93325200717, 0.0410810558113, 1.57584167586, 4.93325200717, 50.0130674801;
  Eigen::Matrix<double, 2, 4> K;
  K << 128.210620632, 6.93828382074, 5.11036686771, 0.729493080206, 5.1103668677, 0.729493080206, 128.210620632, 6.93828382074;
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
