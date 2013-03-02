#include "frc971/control_loops/shooter_motor_plant.h"

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {


StateFeedbackPlant<2, 1, 1> MakeShooterPlant() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.0098571228289, 0.0, 0.971561310859;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.00785005397639, 1.56249765488;
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

StateFeedbackLoop<2, 1, 1> MakeShooterLoop() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.07156131086, 28.0940195016;
  Eigen::Matrix<double, 1, 2> K;
  K << 43.5200653183, 0.819154156845;
  return StateFeedbackLoop<2, 1, 1>(L, K, MakeShooterPlant());
}

}  // namespace frc971
}  // namespace control_loops
