#include "y2015_bot3/control_loops/drivetrain/polydrivetrain_cim_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace y2015_bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<1, 1, 1> MakeCIMPlantCoefficients() {
  Eigen::Matrix<double, 1, 1> A;
  A << 0.783924473544;
  Eigen::Matrix<double, 1, 1> B;
  B << 8.94979586973;
  Eigen::Matrix<double, 1, 1> C;
  C << 1;
  Eigen::Matrix<double, 1, 1> D;
  D << 0;
  Eigen::Matrix<double, 1, 1> U_max;
  U_max << 12.0;
  Eigen::Matrix<double, 1, 1> U_min;
  U_min << -12.0;
  return StateFeedbackPlantCoefficients<1, 1, 1>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<1, 1, 1> MakeCIMController() {
  Eigen::Matrix<double, 1, 1> L;
  L << 0.773924473544;
  Eigen::Matrix<double, 1, 1> K;
  K << 0.086473980503;
  Eigen::Matrix<double, 1, 1> A_inv;
  A_inv << 1.2756330919;
  return StateFeedbackController<1, 1, 1>(L, K, A_inv, MakeCIMPlantCoefficients());
}

StateFeedbackPlant<1, 1, 1> MakeCIMPlant() {
  ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<1, 1, 1>>> plants(1);
  plants[0] = ::std::unique_ptr<StateFeedbackPlantCoefficients<1, 1, 1>>(new StateFeedbackPlantCoefficients<1, 1, 1>(MakeCIMPlantCoefficients()));
  return StateFeedbackPlant<1, 1, 1>(&plants);
}

StateFeedbackLoop<1, 1, 1> MakeCIMLoop() {
  ::std::vector< ::std::unique_ptr<StateFeedbackController<1, 1, 1>>> controllers(1);
  controllers[0] = ::std::unique_ptr<StateFeedbackController<1, 1, 1>>(new StateFeedbackController<1, 1, 1>(MakeCIMController()));
  return StateFeedbackLoop<1, 1, 1>(&controllers);
}

}  // namespace control_loops
}  // namespace y2015_bot3
