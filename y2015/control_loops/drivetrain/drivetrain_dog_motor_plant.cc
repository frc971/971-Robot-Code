#include "y2015/control_loops/drivetrain/drivetrain_dog_motor_plant.h"

#include <vector>

#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {

StateFeedbackPlantCoefficients<4, 2, 2> MakeDrivetrainLowLowPlantCoefficients() {
  Eigen::Matrix<double, 4, 4> A;
  A << 1.0, 0.00494348612824, 0.0, 1.24048455404e-05, 0.0, 0.977484092703, 0.0, 0.00492433284048, 0.0, 1.24048455404e-05, 1.0, 0.00494348612824, 0.0, 0.00492433284048, 0.0, 0.977484092703;
  Eigen::Matrix<double, 4, 2> B;
  B << 2.09180756177e-05, -4.59153636043e-06, 0.00833405032795, -0.00182269526971, -4.59153636043e-06, 2.09180756177e-05, -0.00182269526971, 0.00833405032795;
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
  A << 1.0, 0.00494348612824, 0.0, 1.24048455404e-05, 0.0, 0.977484092703, 0.0, 0.00492433284048, 0.0, 1.24048455404e-05, 1.0, 0.00494348612824, 0.0, 0.00492433284048, 0.0, 0.977484092703;
  Eigen::Matrix<double, 4, 2> B;
  B << 2.09180756177e-05, -4.59153636043e-06, 0.00833405032795, -0.00182269526971, -4.59153636043e-06, 2.09180756177e-05, -0.00182269526971, 0.00833405032795;
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
  A << 1.0, 0.00494348612824, 0.0, 1.24048455404e-05, 0.0, 0.977484092703, 0.0, 0.00492433284048, 0.0, 1.24048455404e-05, 1.0, 0.00494348612824, 0.0, 0.00492433284048, 0.0, 0.977484092703;
  Eigen::Matrix<double, 4, 2> B;
  B << 2.09180756177e-05, -4.59153636043e-06, 0.00833405032795, -0.00182269526971, -4.59153636043e-06, 2.09180756177e-05, -0.00182269526971, 0.00833405032795;
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
  A << 1.0, 0.00494348612824, 0.0, 1.24048455404e-05, 0.0, 0.977484092703, 0.0, 0.00492433284048, 0.0, 1.24048455404e-05, 1.0, 0.00494348612824, 0.0, 0.00492433284048, 0.0, 0.977484092703;
  Eigen::Matrix<double, 4, 2> B;
  B << 2.09180756177e-05, -4.59153636043e-06, 0.00833405032795, -0.00182269526971, -4.59153636043e-06, 2.09180756177e-05, -0.00182269526971, 0.00833405032795;
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
  L << 1.2774840927, 0.00492433284048, 79.1440456622, 1.05150722636, 0.00492433284048, 1.2774840927, 1.05150722636, 79.1440456622;
  Eigen::Matrix<double, 2, 4> K;
  K << 160.19677247, 15.5358468938, 1.60401194139, 1.33861213093, 1.60401194142, 1.33861213094, 160.19677247, 15.5358468938;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00505742153256, 0.0, 1.27875036472e-05, 0.0, 1.02306051542, 0.0, -0.00515393603998, 0.0, 1.27875036472e-05, 1.0, -0.00505742153256, 0.0, -0.00515393603998, 0.0, 1.02306051542;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainLowHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.2774840927, 0.00492433284048, 79.1440456622, 1.05150722636, 0.00492433284048, 1.2774840927, 1.05150722636, 79.1440456622;
  Eigen::Matrix<double, 2, 4> K;
  K << 160.19677247, 15.5358468938, 1.60401194139, 1.33861213093, 1.60401194142, 1.33861213094, 160.19677247, 15.5358468938;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00505742153256, 0.0, 1.27875036472e-05, 0.0, 1.02306051542, 0.0, -0.00515393603998, 0.0, 1.27875036472e-05, 1.0, -0.00505742153256, 0.0, -0.00515393603998, 0.0, 1.02306051542;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainLowHighPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighLowController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.2774840927, 0.00492433284048, 79.1440456622, 1.05150722636, 0.00492433284048, 1.2774840927, 1.05150722636, 79.1440456622;
  Eigen::Matrix<double, 2, 4> K;
  K << 160.19677247, 15.5358468938, 1.60401194139, 1.33861213093, 1.60401194142, 1.33861213094, 160.19677247, 15.5358468938;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00505742153256, 0.0, 1.27875036472e-05, 0.0, 1.02306051542, 0.0, -0.00515393603998, 0.0, 1.27875036472e-05, 1.0, -0.00505742153256, 0.0, -0.00515393603998, 0.0, 1.02306051542;
  return StateFeedbackController<4, 2, 2>(L, K, A_inv, MakeDrivetrainHighLowPlantCoefficients());
}

StateFeedbackController<4, 2, 2> MakeDrivetrainHighHighController() {
  Eigen::Matrix<double, 4, 2> L;
  L << 1.2774840927, 0.00492433284048, 79.1440456622, 1.05150722636, 0.00492433284048, 1.2774840927, 1.05150722636, 79.1440456622;
  Eigen::Matrix<double, 2, 4> K;
  K << 160.19677247, 15.5358468938, 1.60401194139, 1.33861213093, 1.60401194142, 1.33861213094, 160.19677247, 15.5358468938;
  Eigen::Matrix<double, 4, 4> A_inv;
  A_inv << 1.0, -0.00505742153256, 0.0, 1.27875036472e-05, 0.0, 1.02306051542, 0.0, -0.00515393603998, 0.0, 1.27875036472e-05, 1.0, -0.00505742153256, 0.0, -0.00515393603998, 0.0, 1.02306051542;
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
