#include "frc971/control_loops/fridge/arm_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeArmPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00385609864291, 0.0, 0.0, 0.0, 0.580316842139, 0.0, 0.0, 0.0, 0.0, 0.989675611002, 0.00384038140364, 0.0, 0.0, -3.77990295634, 0.571703057879;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000138504938452, 0.000138504938452, 0.050815736504, 0.050815736504, 0.000138231861567, -0.000138231861567, 0.0506086144454, -0.0506086144454;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 1, 0, 1, 0, -1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<4, 2, 2> MakeArmController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 0.59015842107, 0.59015842107, 19.0789855292, 19.0789855292, 0.58068933444, -0.58068933444, 16.4237455811, -16.4237455811;
  Eigen::Matrix<double, 2, 4> K;
  K << 132.284914985, 5.45575813704, 200.395433435, 6.0775850354, 132.284914985, 5.45575813705, -200.395433435, -6.0775850354;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00664481600894, 0.0, 0.0, 0.0, 1.72319658398, 0.0, 0.0, 0.0, 0.0, 0.985156756388, -0.00661773211593, 0.0, 0.0, 6.51351586213, 1.70540563213;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeArmPlantCoefficients());
}

StateFeedbackPlant<4, 2, 2> MakeArmPlant() {
  ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>> plants(1);
  plants[0] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeArmPlantCoefficients()));
  return StateFeedbackPlant<4, 2, 2>(&plants);
}

StateFeedbackLoop<4, 2, 2> MakeArmLoop() {
  ::std::vector< ::std::unique_ptr<StateFeedbackController<4, 2, 2>>> controllers(1);
  controllers[0] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeArmController()));
  return StateFeedbackLoop<4, 2, 2>(&controllers);
}

}  // namespace control_loops
}  // namespace frc971
