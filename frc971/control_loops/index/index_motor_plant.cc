#include "frc971/control_loops/index/index_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeIndex0DiscPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00832470485812, 0.0, 0.68478614982;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.06201698456, 11.6687573378;
  Eigen::Matrix<double, 1, 2> C;
  C << 1, 0;
  Eigen::Matrix<double, 1, 1> D;
  D << 0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max << 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min << -12.0;
  return StateFeedbackPlantCoefficients<2, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<2, 1, 1> MakeIndex1DiscPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00867533005665, 0.0, 0.747315209983;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.0490373507155, 9.35402266105;
  Eigen::Matrix<double, 1, 2> C;
  C << 1, 0;
  Eigen::Matrix<double, 1, 1> D;
  D << 0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max << 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min << -12.0;
  return StateFeedbackPlantCoefficients<2, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<2, 1, 1> MakeIndex2DiscPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00867533005665, 0.0, 0.747315209983;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.0490373507155, 9.35402266105;
  Eigen::Matrix<double, 1, 2> C;
  C << 1, 0;
  Eigen::Matrix<double, 1, 1> D;
  D << 0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max << 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min << -12.0;
  return StateFeedbackPlantCoefficients<2, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<2, 1, 1> MakeIndex3DiscPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00901822957243, 0.0, 0.810292182273;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.0363437103863, 7.02270693014;
  Eigen::Matrix<double, 1, 2> C;
  C << 1, 0;
  Eigen::Matrix<double, 1, 1> D;
  D << 0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max << 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min << -12.0;
  return StateFeedbackPlantCoefficients<2, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<2, 1, 1> MakeIndex4DiscPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00927953099869, 0.0, 0.859452713637;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.0266707124098, 5.20285570613;
  Eigen::Matrix<double, 1, 2> C;
  C << 1, 0;
  Eigen::Matrix<double, 1, 1> D;
  D << 0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max << 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min << -12.0;
  return StateFeedbackPlantCoefficients<2, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<2, 1, 1> MakeIndex0DiscController() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.58478614982, 48.4122215588;
  Eigen::Matrix<double, 1, 2> K;
  K << 1.90251621122, 0.0460029989298;
  return StateFeedbackController<2, 1, 1>(L, K, MakeIndex0DiscPlantCoefficients());
}

StateFeedbackController<2, 1, 1> MakeIndex1DiscController() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.64731520998, 56.0569452572;
  Eigen::Matrix<double, 1, 2> K;
  K << 2.37331047876, 0.0642434141389;
  return StateFeedbackController<2, 1, 1>(L, K, MakeIndex1DiscPlantCoefficients());
}

StateFeedbackController<2, 1, 1> MakeIndex2DiscController() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.64731520998, 56.0569452572;
  Eigen::Matrix<double, 1, 2> K;
  K << 2.37331047876, 0.0642434141389;
  return StateFeedbackController<2, 1, 1>(L, K, MakeIndex2DiscPlantCoefficients());
}

StateFeedbackController<2, 1, 1> MakeIndex3DiscController() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.71029218227, 64.1044007344;
  Eigen::Matrix<double, 1, 2> K;
  K << 3.16117420545, 0.0947502706704;
  return StateFeedbackController<2, 1, 1>(L, K, MakeIndex3DiscPlantCoefficients());
}

StateFeedbackController<2, 1, 1> MakeIndex4DiscController() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.75945271364, 70.6153894746;
  Eigen::Matrix<double, 1, 2> K;
  K << 4.26688750446, 0.137549804289;
  return StateFeedbackController<2, 1, 1>(L, K, MakeIndex4DiscPlantCoefficients());
}

StateFeedbackPlant<2, 1, 1> MakeIndexPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<2, 1, 1> *> plants(5);
  plants[0] = new StateFeedbackPlantCoefficients<2, 1, 1>(MakeIndex0DiscPlantCoefficients());
  plants[1] = new StateFeedbackPlantCoefficients<2, 1, 1>(MakeIndex1DiscPlantCoefficients());
  plants[2] = new StateFeedbackPlantCoefficients<2, 1, 1>(MakeIndex2DiscPlantCoefficients());
  plants[3] = new StateFeedbackPlantCoefficients<2, 1, 1>(MakeIndex3DiscPlantCoefficients());
  plants[4] = new StateFeedbackPlantCoefficients<2, 1, 1>(MakeIndex4DiscPlantCoefficients());
  return StateFeedbackPlant<2, 1, 1>(plants);
}

StateFeedbackLoop<2, 1, 1> MakeIndexLoop() {
  ::std::vector<StateFeedbackController<2, 1, 1> *> controllers(5);
  controllers[0] = new StateFeedbackController<2, 1, 1>(MakeIndex0DiscController());
  controllers[1] = new StateFeedbackController<2, 1, 1>(MakeIndex1DiscController());
  controllers[2] = new StateFeedbackController<2, 1, 1>(MakeIndex2DiscController());
  controllers[3] = new StateFeedbackController<2, 1, 1>(MakeIndex3DiscController());
  controllers[4] = new StateFeedbackController<2, 1, 1>(MakeIndex4DiscController());
  return StateFeedbackLoop<2, 1, 1>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
