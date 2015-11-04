#include "y2014_bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace y2014_bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00488249769672, 0.0, 2.55881088306e-05, 0.0, 0.953388055571, 0.0, 0.0100729449137, 0.0, 2.55881088306e-05, 1.0, 0.00488249769672, 0.0, 0.0100729449137, 0.0, 0.953388055571;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.0031863249e-05, -6.5399448668e-06, 0.0119133285199, -0.00257449680311, -6.5399448668e-06, 3.0031863249e-05, -0.00257449680311, 0.0119133285199;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 0, 0, 1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowHighPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00488249769672, 0.0, 2.55881088306e-05, 0.0, 0.953388055571, 0.0, 0.0100729449137, 0.0, 2.55881088306e-05, 1.0, 0.00488249769672, 0.0, 0.0100729449137, 0.0, 0.953388055571;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.0031863249e-05, -6.5399448668e-06, 0.0119133285199, -0.00257449680311, -6.5399448668e-06, 3.0031863249e-05, -0.00257449680311, 0.0119133285199;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 0, 0, 1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainHighLowPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00488249769672, 0.0, 2.55881088306e-05, 0.0, 0.953388055571, 0.0, 0.0100729449137, 0.0, 2.55881088306e-05, 1.0, 0.00488249769672, 0.0, 0.0100729449137, 0.0, 0.953388055571;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.0031863249e-05, -6.5399448668e-06, 0.0119133285199, -0.00257449680311, -6.5399448668e-06, 3.0031863249e-05, -0.00257449680311, 0.0119133285199;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 0, 0, 1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainHighHighPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00488249769672, 0.0, 2.55881088306e-05, 0.0, 0.953388055571, 0.0, 0.0100729449137, 0.0, 2.55881088306e-05, 1.0, 0.00488249769672, 0.0, 0.0100729449137, 0.0, 0.953388055571;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.0031863249e-05, -6.5399448668e-06, 0.0119133285199, -0.00257449680311, -6.5399448668e-06, 3.0031863249e-05, -0.00257449680311, 0.0119133285199;
  Eigen::Matrix<double, 2, 4> C;
  C << 1, 0, 0, 0, 0, 0, 1, 0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<4, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<4, 2, 2> MakeDrivetrainLowLowController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.25338805557, 0.0100729449137, 74.065541605, 2.10150476024, 0.0100729449137, 1.25338805557, 2.10150476024, 74.065541605;
  Eigen::Matrix<double, 2, 4> K;
  K << 158.010515913, 12.6294601573, 1.98944476188, 1.06520656374, 1.98944476188, 1.06520656374, 158.010515913, 12.6294601573;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00512149525437, 0.0, 2.72716136841e-05, 0.0, 1.04900794038, 0.0, -0.0110832091253, 0.0, 2.72716136841e-05, 1.0, -0.00512149525437, 0.0, -0.0110832091253, 0.0, 1.04900794038;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainLowHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.25338805557, 0.0100729449137, 74.065541605, 2.10150476024, 0.0100729449137, 1.25338805557, 2.10150476024, 74.065541605;
  Eigen::Matrix<double, 2, 4> K;
  K << 158.010515913, 12.6294601573, 1.98944476188, 1.06520656374, 1.98944476188, 1.06520656374, 158.010515913, 12.6294601573;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00512149525437, 0.0, 2.72716136841e-05, 0.0, 1.04900794038, 0.0, -0.0110832091253, 0.0, 2.72716136841e-05, 1.0, -0.00512149525437, 0.0, -0.0110832091253, 0.0, 1.04900794038;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighLowController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.25338805557, 0.0100729449137, 74.065541605, 2.10150476024, 0.0100729449137, 1.25338805557, 2.10150476024, 74.065541605;
  Eigen::Matrix<double, 2, 4> K;
  K << 158.010515913, 12.6294601573, 1.98944476188, 1.06520656374, 1.98944476188, 1.06520656374, 158.010515913, 12.6294601573;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00512149525437, 0.0, 2.72716136841e-05, 0.0, 1.04900794038, 0.0, -0.0110832091253, 0.0, 2.72716136841e-05, 1.0, -0.00512149525437, 0.0, -0.0110832091253, 0.0, 1.04900794038;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.25338805557, 0.0100729449137, 74.065541605, 2.10150476024, 0.0100729449137, 1.25338805557, 2.10150476024, 74.065541605;
  Eigen::Matrix<double, 2, 4> K;
  K << 158.010515913, 12.6294601573, 1.98944476188, 1.06520656374, 1.98944476188, 1.06520656374, 158.010515913, 12.6294601573;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00512149525437, 0.0, 2.72716136841e-05, 0.0, 1.04900794038, 0.0, -0.0110832091253, 0.0, 2.72716136841e-05, 1.0, -0.00512149525437, 0.0, -0.0110832091253, 0.0, 1.04900794038;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainHighHighPlantCoefficients());
}

StateFeedbackPlant<4, 2, 2> MakeDrivetrainPlant() {
  ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>> plants(4);
  plants[0] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainLowLowPlantCoefficients()));
  plants[1] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainLowHighPlantCoefficients()));
  plants[2] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainHighLowPlantCoefficients()));
  plants[3] = ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2>>(new StateFeedbackPlantCoefficients<4, 2, 2>(MakeDrivetrainHighHighPlantCoefficients()));
  return StateFeedbackPlant<4, 2, 2>(&plants);
}

StateFeedbackLoop<4, 2, 2> MakeDrivetrainLoop() {
  ::std::vector< ::std::unique_ptr<StateFeedbackController<4, 2, 2>>> controllers(4);
  controllers[0] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeDrivetrainLowLowController()));
  controllers[1] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeDrivetrainLowHighController()));
  controllers[2] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeDrivetrainHighLowController()));
  controllers[3] = ::std::unique_ptr<StateFeedbackController<4, 2, 2>>(new StateFeedbackController<4, 2, 2>(MakeDrivetrainHighHighController()));
  return StateFeedbackLoop<4, 2, 2>(&controllers);
}

}  // namespace control_loops
}  // namespace y2014_bot3
