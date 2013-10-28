#include "frc971/control_loops/drivetrain/polydrivetrain_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.899177606502, 0.000686937184856, 0.000686937184856, 0.899177606502;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0186835844877, -0.000127297602107, -0.000127297602107, 0.0186835844877;
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
  A << 0.89917740051, 0.000149762601927, 0.000716637450627, 0.978035563679;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0186836226605, -6.07092175404e-05, -0.00013280141337, 0.0089037164526;
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
  A << 0.978035563679, 0.000716637450627, 0.000149762601927, 0.89917740051;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0089037164526, -0.00013280141337, -6.07092175404e-05, 0.0186836226605;
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
  A << 0.978035518136, 0.000156145735499, 0.000156145735499, 0.978035518136;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0089037349145, -6.32967463335e-05, -6.32967463335e-05, 0.0089037349145;
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
  L << 0.879177606502, 0.000686937184856, 0.000686937184856, 0.879177606502;
  Eigen::Matrix<double, 2, 2> K;
  K << 21.3663935118, 0.182343374572, 0.182343374572, 21.3663935118;
  return StateFeedbackController<2, 2, 2>(L, K, MakeVelocityDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainLowHighController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.879178111554, 0.000716636558747, 0.000716636558747, 0.958034852635;
  Eigen::Matrix<double, 2, 2> K;
  K << 21.3663935124, 0.182479162737, 0.399173168486, 53.6921632348;
  return StateFeedbackController<2, 2, 2>(L, K, MakeVelocityDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighLowController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.958040379369, 0.000149803514919, 0.000149803514919, 0.87917258482;
  Eigen::Matrix<double, 2, 2> K;
  K << 53.6921632348, 0.399173168486, 0.182479162737, 21.3663935124;
  return StateFeedbackController<2, 2, 2>(L, K, MakeVelocityDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighHighController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.958035518136, 0.000156145735499, 0.000156145735499, 0.958035518136;
  Eigen::Matrix<double, 2, 2> K;
  K << 53.6921632349, 0.399235265426, 0.399235265426, 53.6921632349;
  return StateFeedbackController<2, 2, 2>(L, K, MakeVelocityDrivetrainHighHighPlantCoefficients());
}

StateFeedbackPlant<2, 2, 2> MakeVDrivetrainPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<2, 2, 2> *> plants(4);
  plants[0] = new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainLowLowPlantCoefficients());
  plants[1] = new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainLowHighPlantCoefficients());
  plants[2] = new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainHighLowPlantCoefficients());
  plants[3] = new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainHighHighPlantCoefficients());
  return StateFeedbackPlant<2, 2, 2>(plants);
}

StateFeedbackLoop<2, 2, 2> MakeVDrivetrainLoop() {
  ::std::vector<StateFeedbackController<2, 2, 2> *> controllers(4);
  controllers[0] = new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainLowLowController());
  controllers[1] = new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainLowHighController());
  controllers[2] = new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainHighLowController());
  controllers[3] = new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainHighHighController());
  return StateFeedbackLoop<2, 2, 2>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
