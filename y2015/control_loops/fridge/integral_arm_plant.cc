#include "y2015/control_loops/fridge/integral_arm_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<5, 2, 2> MakeIntegralArmPlantCoefficients() {
  Eigen::Matrix<double, 5, 5> A;
  A << 1.0, 0.00479642025454, 0.0, 0.0, 4.92993559969e-05, 0.0, 0.919688585028, 0.0, 0.0, 0.0194484035162, 0.0, 0.0, 0.999539771613, 0.00479566382645, 0.0, 0.0, 0.0, -0.18154390621, 0.919241022297, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 5, 2> B;
  B << 2.46496779984e-05, 2.46496779984e-05, 0.00972420175808, 0.00972420175808, 2.46477449538e-05, -2.46477449538e-05, 0.00972266818532, -0.00972266818532, 0.0, 0.0;
  Eigen::Matrix<double, 2, 5> C;
  C << 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<5, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<5, 2, 2> MakeIntegralArmController() {
  Eigen::Matrix<double, 5, 2> L;
  L << 0.461805946837, 0.461805946837, 5.83483983392, 5.83483983392, 0.429725467802, -0.429725467802, 0.18044816586, -0.18044816586, 31.0623964848, 31.0623964848;
  Eigen::Matrix<double, 2, 5> K;
  K << 320.979606093, 21.0129517955, 884.233784759, 36.3637782119, 1.0, 320.979606095, 21.0129517956, -884.233784749, -36.3637782119, 1.0;
  Eigen::Matrix<double, 5, 5> A_inv;
  A_inv << 1.0, -0.00521526561559, 0.0, 0.0, 5.21292341391e-05, 0.0, 1.08732457517, 0.0, 0.0, -0.0211467270909, 0.0, 0.0, 0.999513354044, -0.00521444313273, 0.0, 0.0, 0.0, 0.197397150694, 1.08682415753, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  return StateFeedbackController<5, 2, 2>(L, K, A_inv, MakeIntegralArmPlantCoefficients());
}

StateFeedbackPlant<5, 2, 2> MakeIntegralArmPlant() {
  ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<5, 2, 2>>> plants(1);
  plants[0] = ::std::unique_ptr<StateFeedbackPlantCoefficients<5, 2, 2>>(new StateFeedbackPlantCoefficients<5, 2, 2>(MakeIntegralArmPlantCoefficients()));
  return StateFeedbackPlant<5, 2, 2>(&plants);
}

StateFeedbackLoop<5, 2, 2> MakeIntegralArmLoop() {
  ::std::vector< ::std::unique_ptr<StateFeedbackController<5, 2, 2>>> controllers(1);
  controllers[0] = ::std::unique_ptr<StateFeedbackController<5, 2, 2>>(new StateFeedbackController<5, 2, 2>(MakeIntegralArmController()));
  return StateFeedbackLoop<5, 2, 2>(&controllers);
}

}  // namespace control_loops
}  // namespace frc971
