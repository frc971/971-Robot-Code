#include "bot3/control_loops/elevator/elevator_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeElevatorPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00457884608813, 0.0, 0.836406580139;
  Eigen::Matrix<double, 2, 1> B;
  B << 5.90742803242e-05, 0.0229468687616;
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

StateFeedbackController<2, 1, 1> MakeElevatorController() {
  Eigen::Matrix<double, 2, 1> L;
  L << 0.937593947018, 16.1192495093;
  Eigen::Matrix<double, 1, 2> K;
  K << 597.919635715, 17.5389953523;
  Eigen::Matrix<double, 2, 2> A_inv;
  A_inv << 1.0, -0.0054744261904, 0.0, 1.19559078533;
  return StateFeedbackController<2, 1, 1>(L, K, A_inv, MakeElevatorPlantCoefficients());
}

StateFeedbackPlant<2, 1, 1> MakeElevatorPlant() {
  ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 1, 1>>> plants(1);
  plants[0] = ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 1, 1>>(new StateFeedbackPlantCoefficients<2, 1, 1>(MakeElevatorPlantCoefficients()));
  return StateFeedbackPlant<2, 1, 1>(&plants);
}

StateFeedbackLoop<2, 1, 1> MakeElevatorLoop() {
  ::std::vector< ::std::unique_ptr<StateFeedbackController<2, 1, 1>>> controllers(1);
  controllers[0] = ::std::unique_ptr<StateFeedbackController<2, 1, 1>>(new StateFeedbackController<2, 1, 1>(MakeElevatorController()));
  return StateFeedbackLoop<2, 1, 1>(&controllers);
}

}  // namespace control_loops
}  // namespace bot3
