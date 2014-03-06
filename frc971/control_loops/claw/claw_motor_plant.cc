#include "frc971/control_loops/claw/claw_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeClawPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.0, 0.00807639596609, 0.0, 0.0, 1.0, 0.0, 0.00807639596609, 0.0, 0.0, 0.641687189181, 0.0, 0.0, 0.0, 0.0, 0.641687189181;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000752046077845, 0.0, 0.0, 0.000752046077845, 0.140084829969, 0.0, 0.0, 0.140084829969;
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
  L << 1.60168718918, 2.51306790994e-16, -1.60168718918, 1.60168718918, 47.8568612552, 7.50700456808e-15, -47.8568612552, 47.8568612552;
  Eigen::Matrix<double, 2, 4> K;
  K << 81.0129676169, 0.0, 1.94955302675, 0.0, 0.0, 113.660854272, 0.0, 2.47702820281;
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
