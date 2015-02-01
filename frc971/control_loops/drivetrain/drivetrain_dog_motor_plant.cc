#include "frc971/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00860955515291, 0.0, 0.000228184998733, 0.0, 0.735841675858, 0.0, 0.0410810558113, 0.0, 0.000228184998733, 1.0, 0.00860955515291, 0.0, 0.0410810558113, 0.0, 0.735841675858;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000272244648044, -4.46778919705e-05, 0.0517213538779, -0.00804353916233, -4.46778919705e-05, 0.000272244648044, -0.00804353916233, 0.0517213538779;
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
  A << 1.0, 0.00860667098456, 0.0, 7.04111872002e-05, 0.0, 0.735048848179, 0.0, 0.0131811893199, 0.0, 0.000245343870066, 1.0, 0.00957169266049, 0.0, 0.045929121897, 0.0, 0.915703853642;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000272809358971, -2.57343985847e-05, 0.0518765869984, -0.00481755802263, -4.80375440247e-05, 0.00015654091672, -0.00899277497558, 0.0308091755839;
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
  A << 1.0, 0.00957169266049, 0.0, 0.000245343870066, 0.0, 0.915703853642, 0.0, 0.045929121897, 0.0, 7.04111872002e-05, 1.0, 0.00860667098456, 0.0, 0.0131811893199, 0.0, 0.735048848179;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.00015654091672, -4.80375440247e-05, 0.0308091755839, -0.00899277497558, -2.57343985847e-05, 0.000272809358971, -0.00481755802263, 0.0518765869984;
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
  A << 1.0, 0.00957076892085, 0.0, 7.56192087769e-05, 0.0, 0.915439806567, 0.0, 0.0146814193986, 0.0, 7.56192087769e-05, 1.0, 0.00957076892085, 0.0, 0.0146814193986, 0.0, 0.915439806567;
  Eigen::Matrix<double, 4, 2> B;
  B << 0.000156878531877, -2.76378646165e-05, 0.0309056814511, -0.00536587314624, -2.76378646165e-05, 0.000156878531877, -0.00536587314624, 0.0309056814511;
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
  L << 1.03584167586, 0.0410810558113, 17.1117704011, 3.22861251708, 0.0410810558113, 1.03584167586, 3.22861251708, 17.1117704011;
  Eigen::Matrix<double, 2, 4> K;
  K << 128.210620632, 6.93828382074, 5.11036686771, 0.729493080206, 5.1103668677, 0.729493080206, 128.210620632, 6.93828382074;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.0117194973377, 0.0, 0.000344183176608, 0.0, 1.36323698074, 0.0, -0.0761076958907, 0.0, 0.000344183176608, 1.0, -0.0117194973377, 0.0, -0.0761076958907, 0.0, 1.36323698074;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainLowHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.02891982345, 0.0143715516939, 16.6997472571, 1.23741823594, 0.0143715516939, 1.22183287838, 2.40440177527, 33.5403677132;
  Eigen::Matrix<double, 2, 4> K;
  K << 127.841025245, 6.90618982868, -2.11442482189, 0.171361719101, 11.257083857, 1.47190974842, 138.457761234, 11.0770574926;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.011714710309, 0.0, 9.17355833725e-05, 0.0, 1.36167854796, 0.0, -0.0196008159867, 0.0, 0.00031964754384, 1.0, -0.0104574267731, 0.0, -0.0682979543713, 0.0, 1.09303924439;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighLowController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.21584032636, 0.045928553155, 33.3376290177, 4.12652814156, 0.045928553155, 1.03491237546, 2.45838080322, 16.967272239;
  Eigen::Matrix<double, 2, 4> K;
  K << 138.457761234, 11.0770574926, 11.257083857, 1.47190974842, -2.1144248219, 0.171361719101, 127.841025245, 6.90618982868;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.0104574267731, 0.0, 0.00031964754384, 0.0, 1.09303924439, 0.0, -0.0682979543713, 0.0, 9.17355833725e-05, 1.0, -0.011714710309, 0.0, -0.0196008159867, 0.0, 1.36167854796;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.21543980657, 0.0146814193986, 33.1557840927, 1.47278696694, 0.0146814193986, 1.21543980657, 1.47278696694, 33.1557840927;
  Eigen::Matrix<double, 2, 4> K;
  K << 138.52410152, 11.0779399816, 3.96842371774, 0.882728086516, 3.96842371774, 0.882728086517, 138.52410152, 11.0779399816;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.010456196092, 0.0, 8.50876166887e-05, 0.0, 1.0926521463, 0.0, -0.0175234726538, 0.0, 8.50876166887e-05, 1.0, -0.010456196092, 0.0, -0.0175234726538, 0.0, 1.0926521463;
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
}  // namespace frc971
