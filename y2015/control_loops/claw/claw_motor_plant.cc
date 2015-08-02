#include "y2015/control_loops/claw/claw_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeClawPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00482455476758, 0.0, 0.930652495326;
  Eigen::Matrix<double, 2, 1> B;
  B << 6.97110924671e-05, 0.0275544125308;
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

StateFeedbackController<2, 1, 1> MakeClawController() {
  Eigen::Matrix<double, 2, 1> L;
  L << 0.998251427366, 26.9874526231;
  Eigen::Matrix<double, 1, 2> K;
  K << 74.4310031124, 4.72251126222;
  Eigen::Matrix<double, 2, 2> A_inv;
  A_inv << 1.0, -0.00518405612386, 0.0, 1.07451492907;
  return StateFeedbackController<2, 1, 1>(L, K, A_inv, MakeClawPlantCoefficients());
}

StateFeedbackPlant<2, 1, 1> MakeClawPlant() {
  ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 1, 1>>> plants(1);
  plants[0] = ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 1, 1>>(new StateFeedbackPlantCoefficients<2, 1, 1>(MakeClawPlantCoefficients()));
  return StateFeedbackPlant<2, 1, 1>(&plants);
}

StateFeedbackLoop<2, 1, 1> MakeClawLoop() {
  ::std::vector< ::std::unique_ptr<StateFeedbackController<2, 1, 1>>> controllers(1);
  controllers[0] = ::std::unique_ptr<StateFeedbackController<2, 1, 1>>(new StateFeedbackController<2, 1, 1>(MakeClawController()));
  return StateFeedbackLoop<2, 1, 1>(&controllers);
}

}  // namespace control_loops
}  // namespace frc971
