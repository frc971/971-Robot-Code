#include "bot3/control_loops/elevator/integral_elevator_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<3, 1, 1> MakeIntegralElevatorPlantCoefficients() {
  Eigen::Matrix<double, 3, 3> A;
  A << 1.0, 0.00457884608813, 5.90742803242e-05, 0.0, 0.836406580139, 0.0229468687616, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 3, 1> B;
  B << 5.90742803242e-05, 0.0229468687616, 0.0;
  Eigen::Matrix<double, 1, 3> C;
  C << 1.0, 0.0, 0.0;
  Eigen::Matrix<double, 1, 1> D;
  D << 0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max << 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min << -12.0;
  return StateFeedbackPlantCoefficients<3, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<3, 1, 1> MakeIntegralElevatorController() {
  Eigen::Matrix<double, 3, 1> L;
  L << 0.897632535808, 18.4574278406, 53.6443529697;
  Eigen::Matrix<double, 1, 3> K;
  K << 597.919635715, 17.5389953523, 1.0;
  Eigen::Matrix<double, 3, 3> A_inv;
  A_inv << 1.0, -0.0054744261904, 6.65466590119e-05, 0.0, 1.19559078533, -0.0274350648434, 0.0, 0.0, 1.0;
  return StateFeedbackController<3, 1, 1>(L, K, A_inv, MakeIntegralElevatorPlantCoefficients());
}

StateFeedbackPlant<3, 1, 1> MakeIntegralElevatorPlant() {
  ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<3, 1, 1>>> plants(1);
  plants[0] = ::std::unique_ptr<StateFeedbackPlantCoefficients<3, 1, 1>>(new StateFeedbackPlantCoefficients<3, 1, 1>(MakeIntegralElevatorPlantCoefficients()));
  return StateFeedbackPlant<3, 1, 1>(&plants);
}

StateFeedbackLoop<3, 1, 1> MakeIntegralElevatorLoop() {
  ::std::vector< ::std::unique_ptr<StateFeedbackController<3, 1, 1>>> controllers(1);
  controllers[0] = ::std::unique_ptr<StateFeedbackController<3, 1, 1>>(new StateFeedbackController<3, 1, 1>(MakeIntegralElevatorController()));
  return StateFeedbackLoop<3, 1, 1>(&controllers);
}

}  // namespace control_loops
}  // namespace bot3
