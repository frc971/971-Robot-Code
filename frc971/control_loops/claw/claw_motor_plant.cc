#include "frc971/control_loops/claw/claw_motor_plant.h"

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 1, 1> MakeClawPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00993695674898, 0.0, 0.987417901985;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.00319456032937, 0.637566599616;
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
  L << 1.03584167586, 0.0410810558113;
  Eigen::Matrix<double, 2, 1> K;
  K << 128.210620632, 6.93828382074;
  return StateFeedbackController<2, 1, 1>(L, K, MakeClawPlantCoefficients());
}

StateFeedbackPlant<2, 1, 1> MakeClawPlant() {
  ::std::vector<StateFeedbackPlantCoefficients<2, 1, 1> *> plants(4);
  plants[0] =
      new StateFeedbackPlantCoefficients<2, 1, 1>(MakeClawPlantCoefficients());
  return StateFeedbackPlant<2, 1, 1>(plants);
}

StateFeedbackLoop<2, 1, 1> MakeClawLoop() {
  ::std::vector<StateFeedbackController<2, 1, 1> *> controllers(1);
  controllers[0] = new StateFeedbackController<2, 1, 1>(MakeClawController());
  return StateFeedbackLoop<2, 1, 1>(controllers);
}

}  // namespace frc971
}  // namespace control_loops
