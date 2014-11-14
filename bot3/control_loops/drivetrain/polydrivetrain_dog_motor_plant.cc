#include "bot3/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.498791664199, 0.0897184892451, 0.0897184892451, 0.498791664199;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0720868114296, -0.0129038552514, -0.0129038552514, 0.0720868114296;
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
  A << 0.491841728751, 0.0145551608494, 0.124279814134, 0.921454874639;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0730863931411, -0.00611711060145, -0.0178746738354, 0.0330102308046;
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
  A << 0.921454874639, 0.124279814134, 0.0145551608494, 0.491841728751;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0330102308046, -0.0178746738354, -0.00611711060145, 0.0730863931411;
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
  A << 0.920202836632, 0.0195951135, 0.0195951135, 0.920202836632;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0335364259492, -0.00823525605573, -0.00823525605573, 0.0335364259492;
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
  L << 0.478791664199, 0.0897184892451, 0.0897184892451, 0.478791664199;
  Eigen::Matrix<double, 2, 2> K;
  K << -1.22029287467, 1.0261517351, 1.0261517351, -1.22029287467;
  return StateFeedbackController<2, 2, 2>(L, K, MakeVelocityDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainLowHighController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.450790078471, 0.12008012565, 0.12008012565, 0.922506524919;
  Eigen::Matrix<double, 2, 2> K;
  K << -1.22005303899, 1.06234079364, 3.10424257883, 10.3132835345;
  return StateFeedbackController<2, 2, 2>(L, K, MakeVelocityDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighLowController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.918551885829, 0.0172871978996, 0.0172871978996, 0.454744717561;
  Eigen::Matrix<double, 2, 2> K;
  K << 10.3132835345, 3.10424257883, 1.06234079364, -1.22005303899;
  return StateFeedbackController<2, 2, 2>(L, K, MakeVelocityDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighHighController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.900202836632, 0.0195951135, 0.0195951135, 0.900202836632;
  Eigen::Matrix<double, 2, 2> K;
  K << 10.3132878272, 3.11684016931, 3.11684016931, 10.3132878272;
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
