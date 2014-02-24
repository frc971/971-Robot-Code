#include "frc971/control_loops/claw/claw_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeClawPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.0, 0.00767587925947, 0.0, 0.0, 1.0, 0.0, 0.00767587925947, 0.0, 0.0, 0.574320358283, 0.0, 0.0, 0.0, 0.0, 0.574320358283;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000908630807869, 0.0, 0.0, 0.000908630807869, 0.166422350613, 0.0, 0.0, 0.166422350613;
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
  L << 1.47432035828, 4.62747953155e-16, -1.47432035828, 1.47432035828, 35.823366785, 1.12408806964e-14, -35.823366785, 35.823366785;
  Eigen::Matrix<double, 2, 4> K;
  K << 78.988151683, 0.0, 1.65649264651, 0.0, 0.0, 109.921478378, 0.0, 2.09683545663;
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
