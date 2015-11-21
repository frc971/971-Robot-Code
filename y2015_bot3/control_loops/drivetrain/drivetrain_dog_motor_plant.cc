#include "y2015_bot3/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace y2015_bot3 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.0048650405221, 0.0, 4.30452834469e-05, 0.0, 0.946557122299, 0.0, 0.0169038781857, 0.0, 4.30452834469e-05, 1.0, 0.0048650405221, 0.0, 0.0169038781857, 0.0, 0.946557122299;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.44936607298e-05, -1.10017423477e-05, 0.0136592147549, -0.00432038303814, -1.10017423477e-05, 3.44936607298e-05, -0.00432038303814, 0.0136592147549;
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
  A << 1.0, 0.0048650405221, 0.0, 4.30452834469e-05, 0.0, 0.946557122299, 0.0, 0.0169038781857, 0.0, 4.30452834469e-05, 1.0, 0.0048650405221, 0.0, 0.0169038781857, 0.0, 0.946557122299;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.44936607298e-05, -1.10017423477e-05, 0.0136592147549, -0.00432038303814, -1.10017423477e-05, 3.44936607298e-05, -0.00432038303814, 0.0136592147549;
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
  A << 1.0, 0.0048650405221, 0.0, 4.30452834469e-05, 0.0, 0.946557122299, 0.0, 0.0169038781857, 0.0, 4.30452834469e-05, 1.0, 0.0048650405221, 0.0, 0.0169038781857, 0.0, 0.946557122299;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.44936607298e-05, -1.10017423477e-05, 0.0136592147549, -0.00432038303814, -1.10017423477e-05, 3.44936607298e-05, -0.00432038303814, 0.0136592147549;
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
  A << 1.0, 0.0048650405221, 0.0, 4.30452834469e-05, 0.0, 0.946557122299, 0.0, 0.0169038781857, 0.0, 4.30452834469e-05, 1.0, 0.0048650405221, 0.0, 0.0169038781857, 0.0, 0.946557122299;
  Eigen::Matrix<double, 4, 2> B;
  B << 3.44936607298e-05, -1.10017423477e-05, 0.0136592147549, -0.00432038303814, -1.10017423477e-05, 3.44936607298e-05, -0.00432038303814, 0.0136592147549;
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
  L << 1.2465571223, 0.0169038781857, 72.6644245424, 3.50262182277, 0.0169038781857, 1.2465571223, 3.50262182277, 72.6644245424;
  Eigen::Matrix<double, 2, 4> K;
  K << 156.707528421, 12.1845100817, 3.29243225373, 1.51015663937, 3.29243225372, 1.51015663937, 156.707528421, 12.1845100817;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00514054935697, 0.0, 4.63257162805e-05, 0.0, 1.0567973091, 0.0, -0.01887257785, 0.0, 4.63257162805e-05, 1.0, -0.00514054935697, 0.0, -0.01887257785, 0.0, 1.0567973091;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainLowHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.2465571223, 0.0169038781857, 72.6644245424, 3.50262182277, 0.0169038781857, 1.2465571223, 3.50262182277, 72.6644245424;
  Eigen::Matrix<double, 2, 4> K;
  K << 156.707528421, 12.1845100817, 3.29243225373, 1.51015663937, 3.29243225372, 1.51015663937, 156.707528421, 12.1845100817;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00514054935697, 0.0, 4.63257162805e-05, 0.0, 1.0567973091, 0.0, -0.01887257785, 0.0, 4.63257162805e-05, 1.0, -0.00514054935697, 0.0, -0.01887257785, 0.0, 1.0567973091;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighLowController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.2465571223, 0.0169038781857, 72.6644245424, 3.50262182277, 0.0169038781857, 1.2465571223, 3.50262182277, 72.6644245424;
  Eigen::Matrix<double, 2, 4> K;
  K << 156.707528421, 12.1845100817, 3.29243225373, 1.51015663937, 3.29243225372, 1.51015663937, 156.707528421, 12.1845100817;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00514054935697, 0.0, 4.63257162805e-05, 0.0, 1.0567973091, 0.0, -0.01887257785, 0.0, 4.63257162805e-05, 1.0, -0.00514054935697, 0.0, -0.01887257785, 0.0, 1.0567973091;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.2465571223, 0.0169038781857, 72.6644245424, 3.50262182277, 0.0169038781857, 1.2465571223, 3.50262182277, 72.6644245424;
  Eigen::Matrix<double, 2, 4> K;
  K << 156.707528421, 12.1845100817, 3.29243225373, 1.51015663937, 3.29243225372, 1.51015663937, 156.707528421, 12.1845100817;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00514054935697, 0.0, 4.63257162805e-05, 0.0, 1.0567973091, 0.0, -0.01887257785, 0.0, 4.63257162805e-05, 1.0, -0.00514054935697, 0.0, -0.01887257785, 0.0, 1.0567973091;
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
}  // namespace y2015_bot3
