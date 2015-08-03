#include "y2014/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.735841675858, 0.0410810558113, 0.0410810558113, 0.735841675858;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0517213538779, -0.00804353916233, -0.00804353916233, 0.0517213538779;
  Eigen::Matrix<double, 2, 2> C;
  C << 1.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<2, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainLowHighPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.735048848179, 0.0131811893199, 0.045929121897, 0.915703853642;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0518765869984, -0.00481755802263, -0.00899277497558, 0.0308091755839;
  Eigen::Matrix<double, 2, 2> C;
  C << 1.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<2, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainHighLowPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.915703853642, 0.045929121897, 0.0131811893199, 0.735048848179;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0308091755839, -0.00899277497558, -0.00481755802263, 0.0518765869984;
  Eigen::Matrix<double, 2, 2> C;
  C << 1.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<2, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackPlantCoefficients<2, 2, 2> MakeVelocityDrivetrainHighHighPlantCoefficients() {
  Eigen::Matrix<double, 2, 2> A;
  A << 0.915439806567, 0.0146814193986, 0.0146814193986, 0.915439806567;
  Eigen::Matrix<double, 2, 2> B;
  B << 0.0309056814511, -0.00536587314624, -0.00536587314624, 0.0309056814511;
  Eigen::Matrix<double, 2, 2> C;
  C << 1.0, 0.0, 0.0, 1.0;
  Eigen::Matrix<double, 2, 2> D;
  D << 0.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 2, 1> U_max;
  U_max << 12.0, 12.0;
  Eigen::Matrix<double, 2, 1> U_min;
  U_min << -12.0, -12.0;
  return StateFeedbackPlantCoefficients<2, 2, 2>(A, B, C, D, U_max, U_min);
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainLowLowController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.715841675858, 0.0410810558113, 0.0410810558113, 0.715841675858;
  Eigen::Matrix<double, 2, 2> K;
  K << 2.81809403994, 1.23253744933, 1.23253744933, 2.81809403994;
  Eigen::Matrix<double, 2, 2> A_inv;
  A_inv << 1.36323698074, -0.0761076958907, -0.0761076958907, 1.36323698074;
  return StateFeedbackController<2, 2, 2>(L, K, A_inv, MakeVelocityDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainLowHighController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.715885457343, 0.0459077351335, 0.0459077351335, 0.894867244478;
  Eigen::Matrix<double, 2, 2> K;
  K << 2.81810038978, 1.23928174475, 2.31332592354, 10.6088017388;
  Eigen::Matrix<double, 2, 2> A_inv;
  A_inv << 1.36167854796, -0.0196008159867, -0.0682979543713, 1.09303924439;
  return StateFeedbackController<2, 2, 2>(L, K, A_inv, MakeVelocityDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighLowController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.902328849033, 0.014581304798, 0.014581304798, 0.708423852788;
  Eigen::Matrix<double, 2, 2> K;
  K << 10.6088017388, 2.31332592354, 1.23928174475, 2.81810038978;
  Eigen::Matrix<double, 2, 2> A_inv;
  A_inv << 1.09303924439, -0.0682979543713, -0.0196008159867, 1.36167854796;
  return StateFeedbackController<2, 2, 2>(L, K, A_inv, MakeVelocityDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<2, 2, 2> MakeVelocityDrivetrainHighHighController() {
  Eigen::Matrix<double, 2, 2> L;
  L << 0.895439806567, 0.0146814193986, 0.0146814193986, 0.895439806567;
  Eigen::Matrix<double, 2, 2> K;
  K << 10.6088022944, 2.31694961514, 2.31694961514, 10.6088022944;
  Eigen::Matrix<double, 2, 2> A_inv;
  A_inv << 1.0926521463, -0.0175234726538, -0.0175234726538, 1.0926521463;
  return StateFeedbackController<2, 2, 2>(L, K, A_inv, MakeVelocityDrivetrainHighHighPlantCoefficients());
}

StateFeedbackPlant<2, 2, 2> MakeVelocityDrivetrainPlant() {
  ::std::vector< ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 2, 2>>> plants(4);
  plants[0] = ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 2, 2>>(new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainLowLowPlantCoefficients()));
  plants[1] = ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 2, 2>>(new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainLowHighPlantCoefficients()));
  plants[2] = ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 2, 2>>(new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainHighLowPlantCoefficients()));
  plants[3] = ::std::unique_ptr<StateFeedbackPlantCoefficients<2, 2, 2>>(new StateFeedbackPlantCoefficients<2, 2, 2>(MakeVelocityDrivetrainHighHighPlantCoefficients()));
  return StateFeedbackPlant<2, 2, 2>(&plants);
}

StateFeedbackLoop<2, 2, 2> MakeVelocityDrivetrainLoop() {
  ::std::vector< ::std::unique_ptr<StateFeedbackController<2, 2, 2>>> controllers(4);
  controllers[0] = ::std::unique_ptr<StateFeedbackController<2, 2, 2>>(new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainLowLowController()));
  controllers[1] = ::std::unique_ptr<StateFeedbackController<2, 2, 2>>(new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainLowHighController()));
  controllers[2] = ::std::unique_ptr<StateFeedbackController<2, 2, 2>>(new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainHighLowController()));
  controllers[3] = ::std::unique_ptr<StateFeedbackController<2, 2, 2>>(new StateFeedbackController<2, 2, 2>(MakeVelocityDrivetrainHighHighController()));
  return StateFeedbackLoop<2, 2, 2>(&controllers);
}

}  // namespace control_loops
}  // namespace frc971
