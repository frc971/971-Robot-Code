#include "frc971/control_loops/drivetrain/polydrivetrain_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.8793262727, 0.0205382709865, 0.0205382709865, 0.8793262727;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0223622719242, -0.00380598503859, -0.00380598503859, 0.0223622719242;
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
  A << 0.879138263008, 0.00451814985125, 0.0216200529991, 0.97348145244;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0223971123485, -0.00183152094495, -0.00400645206708, 0.0107498150538;
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
  A << 0.97348145244, 0.0216200529991, 0.00451814985125, 0.879138263008;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0107498150538, -0.00400645206708, -0.00183152094495, 0.0223971123485;
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
  A << 0.973439382316, 0.00475228155545, 0.00475228155545, 0.973439382316;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0107668690064, -0.00192643083821, -0.00192643083821, 0.0107668690064;
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
  L << 0.8593262727, 0.0205382709865, 0.0205382709865, 0.8593262727;
  Eigen::Matrix<double, 2, 2> K;
  K << 13.0245570096, 3.13517071687, 3.13517071687, 13.0245570096;
  return StateFeedbackController<2, 2, 2>(L, K, MakeVelocityDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainLowHighController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.859606856614, 0.0216072038404, 0.0216072038404, 0.953012858834;
  Eigen::Matrix<double, 2, 2> K;
  K << 13.0245575441, 3.13849145977, 6.86545006825, 35.9127730204;
  return StateFeedbackController<2, 2, 2>(L, K, MakeVelocityDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighLowController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.956890244313, 0.00522697730081, 0.00522697730081, 0.855729471134;
  Eigen::Matrix<double, 2, 2> K;
  K << 35.9127730204, 6.86545006825, 3.13849145977, 13.0245575441;
  return StateFeedbackController<2, 2, 2>(L, K, MakeVelocityDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighHighController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.953439382316, 0.00475228155545, 0.00475228155545, 0.953439382316;
  Eigen::Matrix<double, 2, 2> K;
  K << 35.9127730463, 6.86696893904, 6.86696893904, 35.9127730463;
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
