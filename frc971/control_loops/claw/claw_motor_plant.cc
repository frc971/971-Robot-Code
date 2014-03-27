#include "frc971/control_loops/claw/claw_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeClawPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.0, 0.00737284608086, 0.0, 0.0, 1.0, -0.00294667339472, 0.00442617268614, 0.0, 0.0, 0.525184383468, 0.0, 0.0, 0.0, -0.380328742836, 0.144855640632;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.00102145540588, 0.0, -0.00102145540588, 0.00216714216844, 0.184611558069, 0.0, -0.184611558069, 0.332485973629;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 1, 1, 0, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<4, 2, 2> MakeClawController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.48518438347, 2.30607329869e-16, -1.48518438347, 1.10485564063, 34.6171964667, 5.33831435952e-15, -34.6171964667, 3.52560374486;
  Eigen::Matrix<double, 2, 4> K;
  K << 104.272994613, 0.0, 1.72618753001, 0.0, 49.1477742369, 129.930293084, -0.546087759204, 0.551235956004;
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
