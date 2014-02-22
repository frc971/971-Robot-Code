#include "frc971/control_loops/claw/claw_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeClawPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.0, 0.00740659366663, 0.0, 0.0, 1.0, 0.0, 0.00740659366663, 0.0, 0.0, 0.530576083967, 0.0, 0.0, 0.0, 0.0, 0.530576083967;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.00101390984157, 0.0, 0.0, 0.00101390984157, 0.183524472124, 0.0, 0.0, 0.183524472124;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 1, 1, 0, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 24.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -24.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<4, 2, 2> MakeClawController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.43057608397, -4.48948312405e-16, -1.43057608397, 1.43057608397, 31.1907717473, -9.79345171104e-15, -31.1907717473, 31.1907717473;
  Eigen::Matrix<double, 2, 4> K;
  K << 110.395400642, 0.0, 2.50425726274, 0.0, 0.0, 170.435941688, 0.0, 2.89797614353;
  return StateFeedbackController<4, 2, 2>(L, K, MakeClawPlantCoefficients());
}

StateFeedbackPlant<4, 2, 2> MakeClawPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<4, 2, 2> *> plants(1);
  plants[0] = new StateFeedbackPlantCoefficients<4, 2, 2>(MakeClawPlantCoefficients());
  return StateFeedbackPlant<4, 2, 2>(plants);
}

StateFeedbackLoop<4, 2, 2> MakeClawLoop() {
  ::std::vector<StateFeedbackController<4, 2, 2> *> controllers(1);
  controllers[0] = new StateFeedbackController<4, 2, 2>(MakeClawController());
  return StateFeedbackLoop<4, 2, 2>(controllers);
}

}  // namespace control_loops
}  // namespace frc971
