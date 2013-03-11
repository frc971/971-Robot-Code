#include "frc971/control_loops/shooter/shooter_motor_plant.h"

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {


StateFeedbackPlant<2, 1, 1> MakeShooterPlant() {
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
  return StateFeedbackPlant<2, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackLoop<2, 1, 1> MakeShooterLoop() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.08741790198, 29.5581442884;
  Eigen::Matrix<double, 1, 2> K;
  K << 1.19203233114, 0.631478943582;
  return StateFeedbackLoop<2, 1, 1>(L, K, MakeShooterPlant());
}

}  // namespace frc971
}  // namespace control_loops
