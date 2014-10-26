#include "bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00751678417107, 0.0, 0.000244815974033, 0.0, 0.548849954683, 0.0, 0.0396601987617, 0.0, 0.000244815974033, 1.0, 0.00751678417107, 0.0, 0.0396601987617, 0.0, 0.548849954683;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000357151105465, -3.52109126974e-05, 0.0648871256127, -0.0057041694345, -3.52109126974e-05, 0.000357151105465, -0.0057041694345, 0.0648871256127;
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

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowHighPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00751193529517, 0.0, 3.41705796399e-05, 0.0, 0.547617329816, 0.0, 0.00612721792429, 0.0, 0.000291766839985, 1.0, 0.00965616939349, 0.0, 0.0523174915529, 0.0, 0.932105674456;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000357848500096, -1.43609003799e-05, 0.0650644091693, -0.00257509141326, -4.19636699414e-05, 0.000144501999664, -0.00752461776602, 0.0285340095421;
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

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainHighLowPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00965616939349, 0.0, 0.000291766839985, 0.0, 0.932105674456, 0.0, 0.0523174915529, 0.0, 3.41705796399e-05, 1.0, 0.00751193529517, 0.0, 0.00612721792429, 0.0, 0.547617329816;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000144501999664, -4.19636699414e-05, 0.0285340095421, -0.00752461776602, -1.43609003799e-05, 0.000357848500096, -0.00257509141326, 0.0650644091693;
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

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainHighHighPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00965542888285, 0.0, 4.04460947997e-05, 0.0, 0.931897839258, 0.0, 0.00790011087396, 0.0, 4.04460947997e-05, 1.0, 0.00965542888285, 0.0, 0.00790011087396, 0.0, 0.931897839258;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000144813214739, -1.69983168063e-05, 0.0286213566286, -0.00332018673511, -1.69983168063e-05, 0.000144813214739, -0.00332018673511, 0.0286213566286;
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

StateFeedbackController<4, 2, 2> MakeDrivetrainLowLowController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 0.848849954683, 0.0396601987617, 5.07410924717, 1.93309188139, 0.0396601987617, 0.848849954683, 1.93309188139, 5.07410924717;
  Eigen::Matrix<double, 2, 4> K;
  K << 122.814750097, 4.68501085975, 3.50201207752, 0.435944310585, 3.50201207752, 0.435944310585, 122.814750097, 4.68501085975;
  return StateFeedbackController<4, 2, 2>(L, K, MakeDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainLowHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 0.84475614898, 0.00630513438581, 4.80790352862, 0.482569304809, 0.00630513438581, 1.23496685529, 1.52827048914, 35.0095906678;
  Eigen::Matrix<double, 2, 4> K;
  K << 122.438502625, 4.65238426191, -3.6222797894, -0.114901477949, 9.82445972218, 1.21630521516, 139.573569983, 11.6848475814;
  return StateFeedbackController<4, 2, 2>(L, K, MakeDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighLowController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.23976956418, 0.051008821609, 35.4898554745, 3.96774253511, 0.051008821609, 0.839953440087, 1.36277593575, 4.73889175169;
  Eigen::Matrix<double, 2, 4> K;
  K << 139.573569983, 11.6848475814, 9.82445972219, 1.21630521516, -3.62227978939, -0.114901477949, 122.438502625, 4.65238426191;
  return StateFeedbackController<4, 2, 2>(L, K, MakeDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.23189783926, 0.00790011087396, 34.8130461883, 0.806392261077, 0.00790011087396, 1.23189783926, 0.806392261077, 34.8130461883;
  Eigen::Matrix<double, 2, 4> K;
  K << 139.67425847, 11.6895445084, 2.52787999423, 0.599798020725, 2.52787999422, 0.599798020724, 139.67425847, 11.6895445084;
  return StateFeedbackController<4, 2, 2>(L, K, MakeDrivetrainHighHighPlantCoefficients());
}

StateFeedbackPlant<4, 2, 2> MakeDrivetrainPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<4, 2, 2> *> plants(4);
  plants[0] = new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainLowLowPlantCoefficients());
  plants[1] = new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainLowHighPlantCoefficients());
  plants[2] = new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainHighLowPlantCoefficients());
  plants[3] = new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainHighHighPlantCoefficients());
  return StateFeedbackPlant<4, 2, 2>(plants);
}

StateFeedbackLoop<4, 2, 2> MakeDrivetrainLoop() {
  ::std::vector<StateFeedbackController<4, 2, 2> *> controllers(4);
  controllers[0] = new StateFeedbackController<4, 2, 2>(MakeDrivetrainLowLowController());
  controllers[1] = new StateFeedbackController<4, 2, 2>(MakeDrivetrainLowHighController());
  controllers[2] = new StateFeedbackController<4, 2, 2>(MakeDrivetrainHighLowController());
  controllers[3] = new StateFeedbackController<4, 2, 2>(MakeDrivetrainHighHighController());
  return StateFeedbackLoop<4, 2, 2>(controllers);
}

}  // namespace control_loops
}  // namespace bot3
