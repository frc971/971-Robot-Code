#include "frc971/control_loops/angle_adjust/angle_adjust_motor_plant.h"

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {


StateFeedbackPlant<2, 1, 1> MakeAngleAdjustPlant() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00135041324202, 0.0, 0.000610875713246;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.00111752476609, 0.129120861909;
  Eigen::Matrix<double, 1, 2> C;
  C << 1, 0;
  Eigen::Matrix<double, 1, 1> D;
  D << 0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max << 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min << -12.0;
  return StateFeedbackPlant<2, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackLoop<2, 1, 1> MakeAngleAdjustLoop() {
  Eigen::Matrix<double, 2, 1> L;
  L << 0.900610875713, 1.85371819524;
  Eigen::Matrix<double, 1, 2> K;
  K << 12.7787251077, -5.83693180893;
  return StateFeedbackLoop<2, 1, 1>(L, K, MakeAngleAdjustPlant());
}

}  // namespace frc971
}  // namespace control_loops
