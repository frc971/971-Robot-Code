#include "y2015/control_loops/fridge/arm_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeArmPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00479642025454, 0.0, 0.0, 0.0, 0.919688585028, 0.0, 0.0, 0.0, 0.0, 0.999539771613, 0.00479566382645, 0.0, 0.0, -0.18154390621, 0.919241022297;
  Eigen::Matrix<double, 4, 2> B;
  B << 2.46496779984e-05, 2.46496779984e-05, 0.00972420175808, 0.00972420175808, 2.46477449538e-05, -2.46477449538e-05, 0.00972266818532, -0.00972266818532;
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
  L << 0.759844292514, 0.759844292514, 54.2541762188, 54.2541762188, 0.759390396955, -0.759390396955, 54.1048167043, -54.1048167043;
  Eigen::Matrix<double, 2, 4> K;
  K << 320.979606093, 21.0129517955, 884.233784759, 36.3637782119, 320.979606095, 21.0129517956, -884.233784749, -36.3637782119;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00521526561559, 0.0, 0.0, 0.0, 1.08732457517, 0.0, 0.0, 0.0, 0.0, 0.999513354044, -0.00521444313273, 0.0, 0.0, 0.197397150694, 1.08682415753;
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
