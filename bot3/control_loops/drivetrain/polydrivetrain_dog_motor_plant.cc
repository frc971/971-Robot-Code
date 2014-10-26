#include "bot3/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.548849954683, 0.0396601987617, 0.0396601987617, 0.548849954683;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0648871256127, -0.0057041694345, -0.0057041694345, 0.0648871256127;
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

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainLowHighPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.547617329816, 0.00612721792429, 0.0523174915529, 0.932105674456;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0650644091693, -0.00257509141326, -0.00752461776602, 0.0285340095421;
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

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainHighLowPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.932105674456, 0.0523174915529, 0.00612721792429, 0.547617329816;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0285340095421, -0.00752461776602, -0.00257509141326, 0.0650644091693;
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

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainHighHighPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.931897839258, 0.00790011087396, 0.00790011087396, 0.931897839258;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0286213566286, -0.00332018673511, -0.00332018673511, 0.0286213566286;
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

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainLowLowController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.528849954683, 0.0396601987617, 0.0396601987617, 0.528849954683;
  Eigen::Matrix<double, 2, 2> K;
  K << -0.740281917714, 0.546140778145, 0.546140778145, -0.740281917714;
  return StateFeedbackController<2, 2, 2>(L, K, MakeVelocityDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainLowHighController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.524323257635, 0.0520813668112, 0.0520813668112, 0.915399746637;
  Eigen::Matrix<double, 2, 2> K;
  K << -0.740249343264, 0.560664230143, 1.6383045686, 11.786792809;
  return StateFeedbackController<2, 2, 2>(L, K, MakeVelocityDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighLowController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.915985613609, 0.00645546174383, 0.00645546174383, 0.523737390662;
  Eigen::Matrix<double, 2, 2> K;
  K << 11.786792809, 1.6383045686, 0.560664230143, -0.740249343264;
  return StateFeedbackController<2, 2, 2>(L, K, MakeVelocityDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighHighController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.911897839258, 0.00790011087396, 0.00790011087396, 0.911897839258;
  Eigen::Matrix<double, 2, 2> K;
  K << 11.7867933868, 1.64333460977, 1.64333460977, 11.7867933868;
  return StateFeedbackController<2, 2, 2>(L, K, MakeVelocityDrivetrainHighHighPlantCoefficients());
}

StateFeedbackPlant<2, 2, 2> MakeVelocityDrivetrainPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<2, 2, 2> *> plants(4);
  plants[0] = new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainLowLowPlantCoefficients());
  plants[1] = new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainLowHighPlantCoefficients());
  plants[2] = new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainHighLowPlantCoefficients());
  plants[3] = new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainHighHighPlantCoefficients());
  return StateFeedbackPlant<2, 2, 2>(plants);
}

StateFeedbackLoop<2, 2, 2> MakeVelocityDrivetrainLoop() {
  ::std::vector<StateFeedbackController<2, 2, 2> *> controllers(4);
  controllers[0] = new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainLowLowController());
  controllers[1] = new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainLowHighController());
  controllers[2] = new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainHighLowController());
  controllers[3] = new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainHighHighController());
  return StateFeedbackLoop<2, 2, 2>(controllers);
}

}  // namespace control_loops
}  // namespace bot3
