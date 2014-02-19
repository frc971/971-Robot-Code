#include "frc971/control_loops/shooter/shooter_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<3, 1, 1> MakeSprungShooterPlantCoefficients() {
  Eigen::Matrix<double, 3, 3> A;
  A << 0.997145287595, 0.00115072867987, 0.000356210952805, -0.322204030364, -0.000199174994385, 0.0402046120149, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 3, 1> B;
  B << 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 1, 3> C;
  C << 1.0, 0.0, 0.0;
  Eigen::Matrix<double, 1, 1> D;
  D << 0.0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max << 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min << -12.0;
  return StateFeedbackPlantCoefficients<3, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<3, 1, 1> MakeShooterPlantCoefficients() {
  Eigen::Matrix<double, 3, 3> A;
  A << 1.0, 0.00115359397892, 0.000356613321821, 0.0, 0.000172163011452, 0.0403047209622, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 3, 1> B;
  B << 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 1, 3> C;
  C << 1.0, 0.0, 0.0;
  Eigen::Matrix<double, 1, 1> D;
  D << 0.0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max << 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min << -12.0;
  return StateFeedbackPlantCoefficients<3, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<3, 1, 1> MakeSprungShooterController() {
  Eigen::Matrix<double, 3, 1> L;
  L << 0.996946112601, 10.71141318, 224.213599484;
  Eigen::Matrix<double, 1, 3> K;
  K << 121.388812879, 5.06126911425, 0.196946112601;
  return StateFeedbackController<3, 1, 1>(L, K, MakeSprungShooterPlantCoefficients());
}

StateFeedbackController<3, 1, 1> MakeShooterController() {
  Eigen::Matrix<double, 3, 1> L;
  L << 1.00017216301, 11.0141047888, 223.935057347;
  Eigen::Matrix<double, 1, 3> K;
  K << 122.81439697, 5.05065025388, 0.200172163011;
  return StateFeedbackController<3, 1, 1>(L, K, MakeShooterPlantCoefficients());
}

StateFeedbackPlant<3, 1, 1> MakeShooterPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<3, 1, 1> *> plants(2);
  plants[0] = new StateFeedbackPlantCoefficients<3, 1, 1>(MakeSprungShooterPlantCoefficients());
  plants[1] = new StateFeedbackPlantCoefficients<3, 1, 1>(MakeShooterPlantCoefficients());
  return StateFeedbackPlant<3, 1, 1>(plants);
}

StateFeedbackLoop<3, 1, 1> MakeShooterLoop() {
  ::std::vector<StateFeedbackController<3, 1, 1> *> controllers(2);
  controllers[0] = new StateFeedbackController<3, 1, 1>(MakeSprungShooterController());
  controllers[1] = new StateFeedbackController<3, 1, 1>(MakeShooterController());
  return StateFeedbackLoop<3, 1, 1>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
