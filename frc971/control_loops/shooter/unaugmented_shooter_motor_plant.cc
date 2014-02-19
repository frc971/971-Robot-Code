#include "frc971/control_loops/shooter/unaugmented_shooter_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeRawSprungShooterPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.997145287595, 0.00115072867987, -0.322204030364, -0.000199174994385;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.000356210952805, 0.0402046120149;
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

StateFeedbackPlantCoefficients<2, 1, 1> MakeRawShooterPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00115359397892, 0.0, 0.000172163011452;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.000356613321821, 0.0403047209622;
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

StateFeedbackController<2, 1, 1> MakeRawSprungShooterController() {
  Eigen::Matrix<double, 2, 1> L;
  L << 0.896946112601, 1.86767549049;
  Eigen::Matrix<double, 1, 2> K;
  K << 743.451871215, -4.17563006819;
  return StateFeedbackController<2, 1, 1>(L, K, MakeRawSprungShooterPlantCoefficients());
}

StateFeedbackController<2, 1, 1> MakeRawShooterController() {
  Eigen::Matrix<double, 2, 1> L;
  L << 0.900172163011, 2.15224193635;
  Eigen::Matrix<double, 1, 2> K;
  K << 750.532425926, -4.15528738406;
  return StateFeedbackController<2, 1, 1>(L, K, MakeRawShooterPlantCoefficients());
}

StateFeedbackPlant<2, 1, 1> MakeRawShooterPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<2, 1, 1> *> plants(2);
  plants[0] = new StateFeedbackPlantCoefficients<2, 1, 1>(MakeRawSprungShooterPlantCoefficients());
  plants[1] = new StateFeedbackPlantCoefficients<2, 1, 1>(MakeRawShooterPlantCoefficients());
  return StateFeedbackPlant<2, 1, 1>(plants);
}

StateFeedbackLoop<2, 1, 1> MakeRawShooterLoop() {
  ::std::vector<StateFeedbackController<2, 1, 1> *> controllers(2);
  controllers[0] = new StateFeedbackController<2, 1, 1>(MakeRawSprungShooterController());
  controllers[1] = new StateFeedbackController<2, 1, 1>(MakeRawShooterController());
  return StateFeedbackLoop<2, 1, 1>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
