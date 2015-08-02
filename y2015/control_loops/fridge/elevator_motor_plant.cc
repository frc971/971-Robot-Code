#include "y2015/control_loops/fridge/elevator_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeElevatorPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00435668193669, 0.0, 0.0, 0.0, 0.754212786054, 0.0, 0.0, 0.0, 0.0, 0.997194498569, 0.00435222083164, 0.0, 0.0, -1.07131589702, 0.751658962986;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.82580284276e-05, 3.82580284276e-05, 0.0146169286307, 0.0146169286307, 3.82387898839e-05, -3.82387898839e-05, 0.0146019613563, -0.0146019613563;
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

StateFeedbackController<4, 2, 2> MakeElevatorController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 0.677106393027, 0.677106393027, 35.5375738607, 35.5375738607, 0.674426730777, -0.674426730777, 34.7138874344, -34.7138874344;
  Eigen::Matrix<double, 2, 4> K;
  K << 321.310606763, 11.7674534233, 601.047935717, 12.6977148843, 321.310606764, 11.7674534233, -601.047935716, -12.6977148843;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00577646258091, 0.0, 0.0, 0.0, 1.32588576923, 0.0, 0.0, 0.0, 0.0, 0.996613922337, -0.00577054766522, 0.0, 0.0, 1.42044250221, 1.32216599481;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeElevatorPlantCoefficients());
}

StateFeedbackPlant<4, 2, 2> MakeElevatorPlant() {
  ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>> plants(1);
  plants[0] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeElevatorPlantCoefficients()));
  return StateFeedbackPlant<4, 2, 2>(&plants);
}

StateFeedbackLoop<4, 2, 2> MakeElevatorLoop() {
  ::std::vector< ::std::unique_ptr<StateFeedbackController<4, 2, 2>>> controllers(1);
  controllers[0] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeElevatorController()));
  return StateFeedbackLoop<4, 2, 2>(&controllers);
}

}  // namespace control_loops
}  // namespace frc971
