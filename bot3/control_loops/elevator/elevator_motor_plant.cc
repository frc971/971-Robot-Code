#include "bot3/control_loops/elevator/elevator_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeElevatorPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00329835431624, 0.0, 0.407009515002;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.000238685884904, 0.0831773970353;
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
  L << 0.843942422954, 1.30830010079;
  Eigen::Matrix<double, 1, 2> K;
  K << 327.58799433, 4.77067036522;
  Eigen::Matrix<double, 2, 2> A_inv;
  A_inv << 1.0, -0.00810387520358, 0.0, 2.4569450176;
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
