#include "frc971/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 2, 2> MakeDogVelocityDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.858140318975, 0.0183781356125, 0.0183781356125, 0.858140318975;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0235210928559, -0.00304719305118, -0.00304719305118, 0.0235210928559;
  Eigen::Matrix<double, 2, 2> C;
  C << 1.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<2, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<2, 2, 2> MakeDogVelocityDrivetrainLowHighPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.857986344025, 0.00408284222985, 0.0195370380139, 0.968532564253;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0235466227252, -0.00148084354724, -0.00323934525959, 0.0114132132839;
  Eigen::Matrix<double, 2, 2> C;
  C << 1.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<2, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<2, 2, 2> MakeDogVelocityDrivetrainHighLowPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.968532564253, 0.0195370380139, 0.00408284222985, 0.857986344025;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0114132132839, -0.00323934525959, -0.00148084354724, 0.0235466227252;
  Eigen::Matrix<double, 2, 2> C;
  C << 1.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<2, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<2, 2, 2> MakeDogVelocityDrivetrainHighHighPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.968497658015, 0.0043351996773, 0.0043351996773, 0.968497658015;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0114258737482, -0.00157237338764, -0.00157237338764, 0.0114258737482;
  Eigen::Matrix<double, 2, 2> C;
  C << 1.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<2, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<2, 2, 2> MakeDogVelocityDrivetrainLowLowController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.838140318975, 0.0183781356125, 0.0183781356125, 0.838140318975;
  Eigen::Matrix<double, 2, 2> K;
  K << 11.2651379288, 2.240762619, 2.240762619, 11.2651379288;
  return StateFeedbackController<2, 2, 2>(L, K, MakeDogVelocityDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeDogVelocityDrivetrainLowHighController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.838267905918, 0.0195319064993, 0.0195319064993, 0.94825100236;
  Eigen::Matrix<double, 2, 2> K;
  K << 11.2651385254, 2.24416423621, 4.90910926672, 32.9269398272;
  return StateFeedbackController<2, 2, 2>(L, K, MakeDogVelocityDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeDogVelocityDrivetrainHighLowController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.950927164662, 0.00446324489791, 0.00446324489791, 0.835591743616;
  Eigen::Matrix<double, 2, 2> K;
  K << 32.9269398272, 4.90910926672, 2.24416423621, 11.2651385254;
  return StateFeedbackController<2, 2, 2>(L, K, MakeDogVelocityDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeDogVelocityDrivetrainHighHighController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.948497658015, 0.0043351996773, 0.0043351996773, 0.948497658015;
  Eigen::Matrix<double, 2, 2> K;
  K << 32.9269398561, 4.91066546682, 4.91066546682, 32.9269398561;
  return StateFeedbackController<2, 2, 2>(L, K, MakeDogVelocityDrivetrainHighHighPlantCoefficients());
}

StateFeedbackPlant<2, 2, 2> MakeVDogDrivetrainPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<2, 2, 2> *> plants(4);
  plants[0] = new StateFeedbackPlantCoefficients<2, 2, 2>(MakeDogVelocityDrivetrainLowLowPlantCoefficients());
  plants[1] = new StateFeedbackPlantCoefficients<2, 2, 2>(MakeDogVelocityDrivetrainLowHighPlantCoefficients());
  plants[2] = new StateFeedbackPlantCoefficients<2, 2, 2>(MakeDogVelocityDrivetrainHighLowPlantCoefficients());
  plants[3] = new StateFeedbackPlantCoefficients<2, 2, 2>(MakeDogVelocityDrivetrainHighHighPlantCoefficients());
  return StateFeedbackPlant<2, 2, 2>(plants);
}

StateFeedbackLoop<2, 2, 2> MakeVDogDrivetrainLoop() {
  ::std::vector<StateFeedbackController<2, 2, 2> *> controllers(4);
  controllers[0] = new StateFeedbackController<2, 2, 2>(MakeDogVelocityDrivetrainLowLowController());
  controllers[1] = new StateFeedbackController<2, 2, 2>(MakeDogVelocityDrivetrainLowHighController());
  controllers[2] = new StateFeedbackController<2, 2, 2>(MakeDogVelocityDrivetrainHighLowController());
  controllers[3] = new StateFeedbackController<2, 2, 2>(MakeDogVelocityDrivetrainHighHighController());
  return StateFeedbackLoop<2, 2, 2>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
