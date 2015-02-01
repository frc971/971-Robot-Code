#include "frc971/control_loops/fridge/elevator_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeElevatorPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00431150605903, 0.0, 0.0, 0.0, 0.737863444837, 0.0, 0.0, 0.0, 0.0, 0.989566706498, 0.00429496790999, 0.0, 0.0, -3.96458576306, 0.728435659238;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.79928442185e-05, 3.79928442185e-05, 0.0144653608576, 0.0144653608576, 3.79213511069e-05, -3.79213511069e-05, 0.0144098743777, -0.0144098743777;
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
  L << 0.668931722418, 0.668931722418, 33.8393453814, 33.8393453814, 0.659001182868, -0.659001182868, 30.8170494953, -30.8170494953;
  Eigen::Matrix<double, 2, 4> K;
  K << 323.659330424, 11.4297509698, 518.471207663, 11.9234359025, 323.659330423, 11.4297509698, -518.471207664, -11.9234359025;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00584323032833, 0.0, 0.0, 0.0, 1.35526432024, 0.0, 0.0, 0.0, 0.0, 0.98722285856, -0.005820816765, 0.0, 0.0, 5.37306162923, 1.34112444982;
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
