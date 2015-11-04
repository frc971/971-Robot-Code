#include "bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00718627304278, 0.0, 0.000575327102319, 0.0, 0.498791664199, 0.0, 0.0897184892451, 0.0, 0.000575327102319, 1.0, 0.00718627304278, 0.0, 0.0897184892451, 0.0, 0.498791664199;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000404687213064, -8.27470202964e-05, 0.0720868114296, -0.0129038552514, -8.27470202964e-05, 0.000404687213064, -0.0129038552514, 0.0720868114296;
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
  A << 1.0, 0.00715806931946, 0.0, 8.27195198739e-05, 0.0, 0.491841728751, 0.0, 0.0145551608494, 0.0, 0.000706303878161, 1.0, 0.00959963582133, 0.0, 0.124279814134, 0.0, 0.921454874639;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000408743642973, -3.47646073582e-05, 0.0730863931411, -0.00611711060145, -0.000101584891631, 0.000168261415116, -0.0178746738354, 0.0330102308046;
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
  A << 1.0, 0.00959963582133, 0.0, 0.000706303878161, 0.0, 0.921454874639, 0.0, 0.124279814134, 0.0, 8.27195198739e-05, 1.0, 0.00715806931946, 0.0, 0.0145551608494, 0.0, 0.491841728751;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000168261415116, -0.000101584891631, 0.0330102308046, -0.0178746738354, -3.47646073582e-05, 0.000408743642973, -0.00611711060145, 0.0730863931411;
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
  A << 1.0, 0.00959512220091, 0.0, 0.000100752776744, 0.0, 0.920202836632, 0.0, 0.0195951135, 0.0, 0.000100752776744, 1.0, 0.00959512220091, 0.0, 0.0195951135, 0.0, 0.920202836632;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000170158358449, -4.23434605168e-05, 0.0335364259492, -0.00823525605573, -4.23434605168e-05, 0.000170158358449, -0.00823525605573, 0.0335364259492;
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
  L << 0.798791664199, 0.0897184892451, 3.57844915691, 3.42875197165, 0.0897184892451, 0.798791664199, 3.42875197165, 3.57844915691;
  Eigen::Matrix<double, 2, 4> K;
  K << 118.526561433, 4.19900632703, 7.79020074229, 0.921948843301, 7.79020074228, 0.921948843301, 118.526561433, 4.19900632703;
  return StateFeedbackController<4, 2, 2>(L, K, MakeDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainLowHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 0.780361183952, 0.0157698223449, 2.31127571816, 1.09059511674, 0.0157698223449, 1.23293541944, 3.02157734826, 34.5051463402;
  Eigen::Matrix<double, 2, 4> K;
  K << 116.74766142, 4.04674599343, -6.38397728262, -0.174173971494, 20.8480994015, 2.49474383494, 135.799605924, 11.0272286082;
  return StateFeedbackController<4, 2, 2>(L, K, MakeDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighLowController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.26735275667, 0.0994717602348, 37.3766125148, 7.14618746927, 0.0994717602348, 0.745943846725, 2.20133516364, 1.9922178483;
  Eigen::Matrix<double, 2, 4> K;
  K << 135.799605924, 11.0272286082, 20.8480994015, 2.49474383494, -6.38397728263, -0.174173971495, 116.74766142, 4.04674599343;
  return StateFeedbackController<4, 2, 2>(L, K, MakeDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.22020283663, 0.0195951135, 33.6437810428, 1.9756574066, 0.0195951135, 1.22020283663, 1.9756574066, 33.6437810428;
  Eigen::Matrix<double, 2, 4> K;
  K << 136.125709167, 11.0402255404, 6.0764292977, 1.24911698877, 6.0764292977, 1.24911698877, 136.125709167, 11.0402255404;
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
