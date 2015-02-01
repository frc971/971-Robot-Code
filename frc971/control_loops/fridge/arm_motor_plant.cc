#include "frc971/control_loops/fridge/arm_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeArmPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00479642025454, 0.0, 0.0, 0.0, 0.919688585028, 0.0, 0.0, 0.0, 0.0, 0.986224520755, 0.00477375853056, 0.0, 0.0, -5.42143988176, 0.906292554417;
  Eigen::Matrix<double, 4, 2> B;
  B << 2.46496779984e-05, 2.46496779984e-05, 0.00972420175808, 0.00972420175808, 2.45917395578e-05, -2.45917395578e-05, 0.0096782576655, -0.0096782576655;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 1, 0, 1, 0, -1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<4, 2, 2> MakeArmController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 0.759844292514, 0.759844292514, 54.2541762188, 54.2541762188, 0.746258537586, -0.746258537586, 49.8002281115, -49.8002281115;
  Eigen::Matrix<double, 2, 4> K;
  K << 320.979606093, 21.0129517956, 245.279302877, 31.6949206914, 320.979606094, 21.0129517956, -245.279302875, -31.6949206915;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00521526561559, 0.0, 0.0, 0.0, 1.08732457517, 0.0, 0.0, 0.0, 0.0, 0.985434166708, -0.00519062496618, 0.0, 0.0, 5.89486481623, 1.07234615805;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeArmPlantCoefficients());
}

StateFeedbackPlant<4, 2, 2> MakeArmPlant() {
  ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>> plants(1);
  plants[0] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeArmPlantCoefficients()));
  return StateFeedbackPlant<4, 2, 2>(&plants);
}

StateFeedbackLoop<4, 2, 2> MakeArmLoop() {
  ::std::vector< ::std::unique_ptr<StateFeedbackController<4, 2, 2>>> controllers(1);
  controllers[0] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeArmController()));
  return StateFeedbackLoop<4, 2, 2>(&controllers);
}

}  // namespace control_loops
}  // namespace frc971
