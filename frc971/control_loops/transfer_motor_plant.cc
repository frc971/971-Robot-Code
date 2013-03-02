#include "frc971/control_loops/transfer_motor_plant.h"

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {


StateFeedbackPlant<2, 1, 1> MakeTransferPlant() {
  Eigen::Matrix<double, 2, 2> A;
  A << 1.0, 0.00867533005665, 0.0, 0.747315209983;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.0490373507155, 9.35402266105;
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

StateFeedbackLoop<2, 1, 1> MakeTransferLoop() {
  Eigen::Matrix<double, 2, 1> L;
  L << 1.64731520998, 56.0569452572;
  Eigen::Matrix<double, 1, 2> K;
  K << 1.06905877421, 0.0368709177253;
  return StateFeedbackLoop<2, 1, 1>(L, K, MakeTransferPlant());
}

}  // namespace frc971
}  // namespace control_loops
