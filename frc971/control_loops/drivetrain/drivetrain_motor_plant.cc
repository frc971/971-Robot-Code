#include "frc971/control_loops/drivetrain/drivetrain_motor_plant.h"

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {


StateFeedbackPlant<4, 2, 2> MakeDrivetrainPlant() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.0081841122497, 0.0, -5.9473473594e-05, 0.0, 0.660289401132, 0.0, -0.0103071702002, 0.0, -5.9473473594e-05, 1.0, 0.0081841122497, 0.0, -0.0103071702002, 0.0, 0.660289401132;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000333031510271, 1.09073596255e-05, 0.0623024929693, 0.00189032194188, 1.09073596255e-05, 0.000333031510271, 0.00189032194188, 0.0623024929693;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 0, 0, 1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlant<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackLoop<4, 2, 2> MakeDrivetrainLoop() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.50028940113, -0.0103071702002, 41.1373728147, -1.1627040905, -0.0103071702002, 1.50028940113, -1.1627040905, 41.1373728147;
  Eigen::Matrix<double, 2, 4> K;
  K << 48.196534943, -0.087263189034, -1.46233261597, -0.163410937407, -1.46233261597, -0.163410937407, 48.196534943, -0.087263189034;
  return StateFeedbackLoop<4, 2, 2>(L, K, MakeDrivetrainPlant());
}

}  // namespace frc971
}  // namespace control_loops
