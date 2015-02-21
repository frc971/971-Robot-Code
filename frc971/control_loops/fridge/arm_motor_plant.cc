#include "frc971/control_loops/fridge/arm_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeArmPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00385609864291, 0.0, 0.0, 0.0, 0.580316842139, 0.0, 0.0, 0.0, 0.0, 0.997415079306, 0.00385216571322, 0.0, 0.0, -0.94787542156, 0.578159966602;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000138504938452, 0.000138504938452, 0.050815736504, 0.050815736504, 0.000138436627927, -0.000138436627927, 0.0507639082867, -0.0507639082867;
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
  L << 0.59015842107, 0.59015842107, 19.0789855292, 19.0789855292, 0.587787522954, -0.587787522954, 18.4121865078, -18.4121865078;
  Eigen::Matrix<double, 2, 4> K;
  K << 132.284914985, 5.45575813704, 227.613831364, 6.12234622766, 132.284914985, 5.45575813704, -227.613831364, -6.12234622766;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00664481600894, 0.0, 0.0, 0.0, 1.72319658398, 0.0, 0.0, 0.0, 0.0, 0.996283279443, -0.00663803879795, 0.0, 0.0, 1.63337568847, 1.71874225747;
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
