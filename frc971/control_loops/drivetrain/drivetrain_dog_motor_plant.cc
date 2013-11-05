#include "frc971/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDogDrivetrainPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00927226867138, 0.0, 9.67653917682e-05, 0.0, 0.858140318975, 0.0, 0.0183781356125, 0.0, 9.67653917682e-05, 1.0, 0.00927226867138, 0.0, 0.0183781356125, 0.0, 0.858140318975;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000120661741455, -1.60442188265e-05, 0.0235210928559, -0.00304719305118, -1.60442188265e-05, 0.000120661741455, -0.00304719305118, 0.0235210928559;
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
  L << 1.69814031898, 0.0183781356125, 65.3030659057, 2.40312922858, 0.0183781356125, 1.69814031898, 2.40312922858, 65.3030659057;
  Eigen::Matrix<double, 2, 4> K;
  K << 257.282562369, 15.135374919, 33.3313439577, 2.7466676405, 33.3313439577, 2.7466676405, 257.282562369, 15.135374919;
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
