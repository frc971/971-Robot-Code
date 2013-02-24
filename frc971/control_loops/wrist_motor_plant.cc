#include "frc971/control_loops/wrist_motor_plant.h"

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {


StateFeedbackPlant<2, 1, 1> MakeWristPlant() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00688850240086, 0.0, 0.450098411557;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.00106724827203, 0.188617057012;
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

StateFeedbackLoop<2, 1, 1> MakeWristLoop() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.35009841156, 23.2478308944;
  Eigen::Matrix<double, 1, 2> K;
  K << 8.74788328338, -1.58648298569;
  return StateFeedbackLoop<2, 1, 1>(L, K, MakeWristPlant());
}

}  // namespace frc971
}  // namespace control_loops
